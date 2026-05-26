"""Read DTCs from all emission-related ECUs via standard OBD-II over CAN
(ISO 15765-4 / SAE J1979) using a J2534 interface.

Services used:
  Mode 03 (0x03) - Request stored emission-related DTCs
  Mode 07 (0x07) - Request pending emission-related DTCs
  Mode 0A (0x0A) - Request permanent emission-related DTCs
  Mode 04 (0x04) - Clear DTCs (optional, prompts before executing)

The functional request address 0x7DF is broadcast to all ECUs;
each responding ECU at 0x7E8..0x7EF is then queried individually.

Uses Shamtam/PyJ2534 (https://github.com/Shamtam/PyJ2534).
"""

import os
import sys
import time
import struct

from PyJ2534.PyJ2534 import (
    get_interfaces, load_interface,
    ProtocolID, FilterType, TxFlags,
    PASSTHRU_MSG,
)
from PyJ2534.PyJ2534.error import J2534Error


# ── OBD-II CAN addressing (ISO 15765-4, 11-bit IDs) ──────────────────────────
#
# 0x7DF  = OBD-II functional / broadcast request (all ECUs listen)
# 0x7E0..0x7E7 = physical request  to ECU slots 1..8
# 0x7E8..0x7EF = physical response from ECU slots 1..8

OBD_FUNCTIONAL_TX = 0x7DF          # broadcast to all ECUs
OBD_ECU_SLOTS = [                  # (tx_physical, rx_physical)
    # ── Standard OBD-II (ISO 15765-4 / SAE J1979) ────────────────────────────
    # Slots 1–8 are defined by the standard; every compliant ECU uses these.
    (0x7E0, 0x7E8),   # slot 1 – ECM / PCM              [all OEMs]
    (0x7E1, 0x7E9),   # slot 2 – TCM                    [all OEMs]
    (0x7E2, 0x7EA),   # slot 3                           [HKMC: FATC/HVAC]
    (0x7E3, 0x7EB),   # slot 4                           [HKMC: DATC/manual A/C]
    (0x7E4, 0x7EC),   # slot 5                           [HKMC: EPB/VDC]
    (0x7E5, 0x7ED),   # slot 6                           [HKMC: ACU/hybrid]
    (0x7E6, 0x7EE),   # slot 7                           [HKMC: FCA/ADAS]
    (0x7E7, 0x7EF),   # slot 8                           [HKMC: CGW/gateway]
    (0x7DF, 0x7E8),   # OBD-II functional → ECM response
]

# Map slot response IDs to friendly names for well-known assignments.
ECU_SLOT_NAMES = {
    0x7E8: "ECM/PCM (engine)",
    0x7E9: "TCM (transmission)",
    0x7EA: "slot 3",
    0x7EB: "slot 4",
    0x7EC: "slot 5",
    0x7ED: "slot 6",
    0x7EE: "slot 7",
    0x7EF: "slot 8",
}

# DTC system identifier characters (ISO 15031-6, first nibble bits 7-6).
DTC_SYSTEM = {0b00: 'P', 0b01: 'C', 0b10: 'B', 0b11: 'U'}


# ── Helpers ───────────────────────────────────────────────────────────────────

def hexs(data):
    return ' '.join(f'{b:02X}' for b in data)


def get_msg_bytes(msg):
    data = bytes(msg.Data or b'')
    return data if data else bytes(msg.ExtraData or b'')


def build_msg(txid, payload):
    """Build an ISO 15765 PASSTHRU_MSG for the given CAN ID + payload."""
    data = struct.pack('>I', txid) + bytes(payload)
    return PASSTHRU_MSG(
        ProtocolID.ISO15765,
        tx_flags=TxFlags.ISO15765_FRAME_PAD,
        data=data,
    )


def setup_flow_filter(iface, channel, txid, rxid):
    """Register a flow-control filter so the J2534 stack reassembles
    multi-frame responses from the given (tx, rx) pair.
    """
    mask = PASSTHRU_MSG(
        ProtocolID.ISO15765,
        tx_flags=TxFlags.ISO15765_FRAME_PAD,
        data=b'\xFF\xFF\xFF\xFF',
    )
    pattern = PASSTHRU_MSG(
        ProtocolID.ISO15765,
        tx_flags=TxFlags.ISO15765_FRAME_PAD,
        data=struct.pack('>I', rxid),
    )
    flow = PASSTHRU_MSG(
        ProtocolID.ISO15765,
        tx_flags=TxFlags.ISO15765_FRAME_PAD,
        data=struct.pack('>I', txid),
    )
    return iface.PassThruStartMsgFilter(
        channel_id=channel,
        filter_type=FilterType.FLOW_CONTROL_FILTER,
        mask_msg=mask, pattern_msg=pattern, flow_msg=flow,
    )


def transact(iface, channel, txid, rxid, payload, timeout_ms=1000):
    """Send a single OBD-II / UDS request and return the first valid
    application-layer response payload (sans 4-byte CAN prefix), or None.
    Handles UDS 0x7F xx 0x78 (responsePending) by extending the deadline.
    """
    iface.PassThruWriteMsgs(channel_id=channel,
                             msgs=[build_msg(txid, payload)],
                             timeout=200)

    deadline = time.time() + timeout_ms / 1000.0
    while time.time() < deadline:
        try:
            msgs = iface.PassThruReadMsgs(channel_id=channel,
                                          num_msgs=1, timeout=200)
        except J2534Error:
            msgs = []

        for m in msgs:
            raw = get_msg_bytes(m)
            if len(raw) < 5:
                continue
            rid = struct.unpack('>I', raw[:4])[0]
            data = raw[4:]

            if int(m._RxStatus) & 0x09:    # TX echo / loopback
                continue
            if rid != rxid:
                continue

            # responsePending – ECU is still processing.
            if len(data) >= 3 and data[0] == 0x7F and data[2] == 0x78:
                deadline = time.time() + 5.0
                continue

            return data
    return None


# ── DTC decoding (ISO 15031-6) ────────────────────────────────────────────────

def decode_dtc_pair(b1, b2):
    """Decode a 2-byte OBD-II DTC (ISO 15031-6) into e.g. "P0420"."""
    system  = DTC_SYSTEM[(b1 >> 6) & 0x03]
    digit1  = (b1 >> 4) & 0x03       # second character (0-3)
    digit2  = b1 & 0x0F
    return f"{system}{digit1}{digit2:X}{b2:02X}"


def parse_dtc_response(resp, positive_sid):
    """Parse a Mode-03/07/0A response body into a list of DTC strings.

    Format (ISO 15765-4 PCI already stripped by J2534 layer):
        <positive_SID> <count> [ DTC_H DTC_L ] ...
    Some ECUs omit the count byte; we handle both.
    """
    if resp is None or len(resp) == 0:
        return None  # no response
    if resp[0] == 0x7F:
        return []    # negative response → service not supported / no DTCs

    if resp[0] != positive_sid:
        return []    # unexpected SID

    body = resp[1:]  # skip positive SID byte
    dtcs = []

    # Some ECUs include a DTC count byte; others go straight to pairs.
    if len(body) >= 1:
        count_candidate = body[0]
        if len(body) >= 1 + count_candidate * 2:
            body = body[1:]  # skip count byte

    for i in range(0, len(body) - 1, 2):
        b1, b2 = body[i], body[i + 1]
        if b1 == 0x00 and b2 == 0x00:
            continue  # padding / empty slot
        dtcs.append(decode_dtc_pair(b1, b2))
    return dtcs


# ── OBD-II service wrappers ───────────────────────────────────────────────────

def discover_ecus(iface, channel):
    """Broadcast OBD-II Mode 01 PID 00 (supported PIDs) to find all
    responding ECUs.  Returns list of (txid_physical, rxid_physical).
    """
    print("\nDiscovering ECUs via OBD-II functional request (Mode 01, PID 00)...")

    # Open a pass-all filter to collect any response in 0x7E8..0x7EF.
    mask = PASSTHRU_MSG(
        ProtocolID.ISO15765,
        tx_flags=TxFlags.ISO15765_FRAME_PAD,
        data=b'\xFF\xF8\xFF\xFF',          # mask out low 3 bits of ID
    )
    pattern = PASSTHRU_MSG(
        ProtocolID.ISO15765,
        tx_flags=TxFlags.ISO15765_FRAME_PAD,
        data=b'\x00\x00\x07\xE8',          # 0x7E8 base
    )
    try:
        fid = iface.PassThruStartMsgFilter(
            channel_id=channel,
            filter_type=FilterType.PASS_FILTER,
            mask_msg=mask, pattern_msg=pattern,
        )
    except J2534Error:
        # Fallback: use individual flow-control filters per slot.
        fid = None

    # Send functional request.
    iface.PassThruWriteMsgs(
        channel_id=channel,
        msgs=[build_msg(OBD_FUNCTIONAL_TX, b'\x01\x00')],
        timeout=200,
    )

    responding_rxids = set()
    deadline = time.time() + 1.5
    while time.time() < deadline:
        try:
            msgs = iface.PassThruReadMsgs(channel_id=channel,
                                          num_msgs=10, timeout=200)
        except J2534Error:
            msgs = []
        for m in msgs:
            raw = get_msg_bytes(m)
            if len(raw) < 5:
                continue
            if int(m._RxStatus) & 0x09:
                continue
            rid = struct.unpack('>I', raw[:4])[0]
            if 0x7E8 <= rid <= 0x7EF:
                responding_rxids.add(rid)

    if fid is not None:
        try:
            iface.PassThruStopMsgFilter(channel_id=channel, filter_id=fid)
        except J2534Error:
            pass

    found = []
    for rxid in sorted(responding_rxids):
        txid = rxid - 8                   # 0x7E8 → 0x7E0, 0x7E9 → 0x7E1, …
        name = ECU_SLOT_NAMES.get(rxid, f"unknown (RX=0x{rxid:03X})")
        print(f"  Found ECU: TX=0x{txid:03X}  RX=0x{rxid:03X}  [{name}]")
        found.append((txid, rxid))

    if not found:
        print("  No ECU responded to functional broadcast.")
        # Fall back to trying all standard slots individually.
        print("  Falling back to individual slot probing...")
        for txid, rxid in OBD_ECU_SLOTS:
            try:
                fid2 = setup_flow_filter(iface, channel, txid, rxid)
            except J2534Error as e:
                print(f"    filter error TX=0x{txid:03X}: {e}")
                continue
            try:
                resp = transact(iface, channel, txid, rxid,
                                b'\x01\x00', timeout_ms=400)
            finally:
                try:
                    iface.PassThruStopMsgFilter(channel_id=channel, filter_id=fid2)
                except J2534Error:
                    pass
            if resp is not None:
                name = ECU_SLOT_NAMES.get(rxid, f"RX=0x{rxid:03X}")
                print(f"  Found ECU: TX=0x{txid:03X}  RX=0x{rxid:03X}  [{name}]")
                found.append((txid, rxid))
    return found


def read_stored_dtcs(iface, channel, txid, rxid):
    """OBD-II Mode 03 – stored (confirmed) emission-related DTCs."""
    resp = transact(iface, channel, txid, rxid, b'\x03', timeout_ms=2000)
    print(f"  Mode 03 raw: {hexs(resp) if resp else 'no response'}")
    dtcs = parse_dtc_response(resp, positive_sid=0x43)
    if dtcs is None:
        print("  Mode 03: no response")
    elif not dtcs:
        print("  Mode 03: no stored DTCs")
    else:
        for code in dtcs:
            print(f"  [STORED]  {code}")
    return dtcs or []


def read_pending_dtcs(iface, channel, txid, rxid):
    """OBD-II Mode 07 – pending (not yet confirmed) DTCs."""
    resp = transact(iface, channel, txid, rxid, b'\x07', timeout_ms=2000)
    print(f"  Mode 07 raw: {hexs(resp) if resp else 'no response'}")
    dtcs = parse_dtc_response(resp, positive_sid=0x47)
    if dtcs is None:
        print("  Mode 07: no response")
    elif not dtcs:
        print("  Mode 07: no pending DTCs")
    else:
        for code in dtcs:
            print(f"  [PENDING]  {code}")
    return dtcs or []


def read_permanent_dtcs(iface, channel, txid, rxid):
    """OBD-II Mode 0A – permanent (non-erasable) DTCs."""
    resp = transact(iface, channel, txid, rxid, b'\x0A', timeout_ms=2000)
    print(f"  Mode 0A raw: {hexs(resp) if resp else 'no response'}")
    dtcs = parse_dtc_response(resp, positive_sid=0x4A)
    if dtcs is None:
        print("  Mode 0A: no response")
    elif not dtcs:
        print("  Mode 0A: no permanent DTCs")
    else:
        for code in dtcs:
            print(f"  [PERMANENT]  {code}")
    return dtcs or []


def clear_dtcs(iface, channel, txid, rxid):
    """OBD-II Mode 04 – clear emission-related DTCs.
    Asks for confirmation before sending.
    """
    confirm = input(
        f"\n  Clear ALL DTCs on ECU TX=0x{txid:03X}?  (yes/no): "
    ).strip().lower()
    if confirm != 'yes':
        print("  Skipped.")
        return
    resp = transact(iface, channel, txid, rxid, b'\x04', timeout_ms=3000)
    if resp is None:
        print("  Mode 04: no response")
    elif resp[0] == 0x44:
        print("  Mode 04: DTCs cleared successfully.")
    elif resp[0] == 0x7F:
        print(f"  Mode 04: negative response: {hexs(resp)}")
    else:
        print(f"  Mode 04: unexpected: {hexs(resp)}")


def read_vin(iface, channel, txid, rxid):
    """OBD-II Mode 09 PID 02 – read VIN (optional, supported by most ECMs)."""
    resp = transact(iface, channel, txid, rxid, b'\x09\x02', timeout_ms=2000)
    if resp is None or resp[0] != 0x49:
        return None
    # Response: 49 02 <count> <17-byte VIN>
    body = resp[3:] if len(resp) >= 3 else resp[1:]
    try:
        vin = body.decode('ascii', errors='replace').strip('\x00').strip()
    except Exception:
        vin = hexs(body)
    return vin or None


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    ifaces = get_interfaces()
    if not ifaces:
        print("No J2534 interface found in registry.")
        sys.exit(1)

    iface_names = list(ifaces.keys())
    iface_paths = list(ifaces.values())
    print(f"Using J2534 interface: {iface_names[0]}")
    print(f"  DLL: {iface_paths[0]}")

    iface = load_interface(iface_paths[0])
    device_id = iface.PassThruOpen()

    channel = iface.PassThruConnect(
        device_id=device_id,
        protocol=ProtocolID.ISO15765,
        flags=0,
        baud=500000,               # 500 kbps – standard OBD-II CAN rate
    )
    print("Connected (ISO15765 @ 500 kbps)")

    all_stored   = {}
    all_pending  = {}
    all_permanent = {}

    try:
        ecus = discover_ecus(iface, channel)
        if not ecus:
            print("No OBD-II ECUs found; exiting.")
            return

        for txid, rxid in ecus:
            name = ECU_SLOT_NAMES.get(rxid, f"RX=0x{rxid:03X}")
            print(f"\n{'='*50}")
            print(f"  ECU: {name}  (TX=0x{txid:03X}  RX=0x{rxid:03X})")
            print(f"{'='*50}")

            try:
                fid = setup_flow_filter(iface, channel, txid, rxid)
            except J2534Error as e:
                print(f"  Could not set flow-control filter: {e}")
                continue

            try:
                # VIN (ECM only – slot 1)
                if rxid == 0x7E8:
                    vin = read_vin(iface, channel, txid, rxid)
                    if vin:
                        print(f"  VIN: {vin}")

                stored    = read_stored_dtcs(iface, channel, txid, rxid)
                pending   = read_pending_dtcs(iface, channel, txid, rxid)
                permanent = read_permanent_dtcs(iface, channel, txid, rxid)

                if stored:
                    all_stored[rxid]    = stored
                if pending:
                    all_pending[rxid]   = pending
                if permanent:
                    all_permanent[rxid] = permanent

            finally:
                try:
                    iface.PassThruStopMsgFilter(channel_id=channel, filter_id=fid)
                except J2534Error:
                    pass

    finally:
        iface.PassThruDisconnect(channel_id=channel)
        iface.PassThruClose(device_id=device_id)

    # ── Summary ───────────────────────────────────────────────────────────────
    print(f"\n{'='*50}")
    print("SUMMARY")
    print(f"{'='*50}")
    if not any([all_stored, all_pending, all_permanent]):
        print("No DTCs found on any ECU.")
    else:
        for rxid, codes in all_stored.items():
            name = ECU_SLOT_NAMES.get(rxid, f"0x{rxid:03X}")
            print(f"  {name} – stored:    {', '.join(codes)}")
        for rxid, codes in all_pending.items():
            name = ECU_SLOT_NAMES.get(rxid, f"0x{rxid:03X}")
            print(f"  {name} – pending:   {', '.join(codes)}")
        for rxid, codes in all_permanent.items():
            name = ECU_SLOT_NAMES.get(rxid, f"0x{rxid:03X}")
            print(f"  {name} – permanent: {', '.join(codes)}")

    print("\nDone.")


if __name__ == '__main__':
    main()
