"""Read DTCs from all emission-related ECUs via standard OBD-II over CAN
(ISO 15765-4 / SAE J1979) using a J2534 interface.

Services used:
  Mode 03 (0x03) - Request stored emission-related DTCs
  Mode 07 (0x07) - Request pending emission-related DTCs
  Mode 0A (0x0A) - Request permanent emission-related DTCs
  Mode 01 (0x01) - Discover ECUs using supported-PID requests
  Mode 02 (0x02) - Read freeze-frame data
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
from PyJ2534.PyJ2534.error import J2534Error, J2534Errors


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

FREEZE_FRAME_COUNT = 16

# PID, label, required data bytes, decoder, unit.
DIAGNOSTIC_PIDS = (
    (0x04, "Calculated engine load", 1, lambda d: d[0] * 100 / 255, "%"),
    (0x05, "Coolant temperature", 1, lambda d: d[0] - 40, "C"),
    (0x0B, "Intake manifold absolute pressure (MAP)", 1, lambda d: d[0], "kPa absolute"),
    (0x0C, "Engine speed", 2, lambda d: (256 * d[0] + d[1]) / 4, "rpm"),
    (0x0D, "Vehicle speed", 1, lambda d: d[0], "km/h"),
    (0x0F, "Intake air temperature", 1, lambda d: d[0] - 40, "C"),
    (0x10, "Mass air flow", 2, lambda d: (256 * d[0] + d[1]) / 100, "g/s"),
    (0x11, "Absolute throttle position", 1, lambda d: d[0] * 100 / 255, "%"),
    (0x33, "Barometric pressure", 1, lambda d: d[0], "kPa absolute"),
)


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


def transact(iface, channel, txid, rxid, payload, timeout_ms=1000,
             expected_prefix=None):
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
        except J2534Error as e:
            if e.error not in (J2534Errors.ERR_TIMEOUT, J2534Errors.ERR_BUFFER_EMPTY):
                raise
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
            if (len(data) >= 3 and data[0] == 0x7F
                    and data[1] == payload[0] and data[2] == 0x78):
                deadline = time.time() + 5.0
                continue

            if expected_prefix is not None:
                is_negative = (len(data) >= 2 and data[0] == 0x7F
                               and data[1] == payload[0])
                if not is_negative and not data.startswith(expected_prefix):
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
    if resp[0] != positive_sid:
        return None  # negative or unexpected response

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
        data=b'\xFF\xFF\xFF\xF8',          # mask out low 3 bits of ID
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
    except J2534Error as e:
        print(f"  Functional discovery filter unavailable: {e}")
        fid = None

    responding_rxids = set()
    if fid is not None:
        try:
            iface.PassThruWriteMsgs(
                channel_id=channel,
                msgs=[build_msg(OBD_FUNCTIONAL_TX, b'\x01\x00')],
                timeout=200,
            )
            deadline = time.time() + 1.5
            while time.time() < deadline:
                try:
                    msgs = iface.PassThruReadMsgs(channel_id=channel,
                                                  num_msgs=1, timeout=200)
                except J2534Error as e:
                    if e.error not in (J2534Errors.ERR_TIMEOUT, J2534Errors.ERR_BUFFER_EMPTY):
                        raise
                    msgs = []
                for m in msgs:
                    raw = get_msg_bytes(m)
                    if len(raw) < 10 or int(m._RxStatus) & 0x09:
                        continue
                    rid = struct.unpack('>I', raw[:4])[0]
                    if 0x7E8 <= rid <= 0x7EF and raw[4:6] == b'\x41\x00':
                        responding_rxids.add(rid)
        finally:
            iface.PassThruStopMsgFilter(channel_id=channel, filter_id=fid)

    found = []
    for rxid in sorted(responding_rxids):
        txid = rxid - 8                   # 0x7E8 → 0x7E0, 0x7E9 → 0x7E1, …
        name = ECU_SLOT_NAMES.get(rxid, f"unknown (RX=0x{rxid:03X})")
        print(f"  Found ECU: TX=0x{txid:03X}  RX=0x{rxid:03X}  [{name}]")
        found.append((txid, rxid))

    if not found:
        if fid is not None:
            print("  No ECU responded to functional broadcast.")
        print("  Falling back to individual slot probing...")
        for txid, rxid in OBD_ECU_SLOTS:
            if txid == OBD_FUNCTIONAL_TX:
                continue
            try:
                fid2 = setup_flow_filter(iface, channel, txid, rxid)
            except J2534Error as e:
                print(f"    filter error TX=0x{txid:03X}: {e}")
                continue
            try:
                for request_id in (txid, OBD_FUNCTIONAL_TX):
                    resp = transact(iface, channel, request_id, rxid,
                                    b'\x01\x00', timeout_ms=1000,
                                    expected_prefix=b'\x41\x00')
                    if resp is not None and len(resp) >= 6 and resp.startswith(b'\x41\x00'):
                        name = ECU_SLOT_NAMES.get(rxid, f"RX=0x{rxid:03X}")
                        print(f"  Found ECU: TX=0x{txid:03X}  RX=0x{rxid:03X}  [{name}]")
                        found.append((txid, rxid))
                        break
                    if resp is not None:
                        print(f"    Probe TX=0x{request_id:03X} RX=0x{rxid:03X}: "
                              f"negative or incomplete response {hexs(resp)}")
                else:
                    print(f"    No valid Mode 01 PID 00 response from RX=0x{rxid:03X}.")
            finally:
                iface.PassThruStopMsgFilter(channel_id=channel, filter_id=fid2)
    return found


def read_stored_dtcs(iface, channel, txid, rxid):
    """OBD-II Mode 03 – stored (confirmed) emission-related DTCs."""
    resp = transact(iface, channel, txid, rxid, b'\x03', timeout_ms=2000)
    print(f"  Mode 03 raw: {hexs(resp) if resp else 'no response'}")
    dtcs = parse_dtc_response(resp, positive_sid=0x43)
    if dtcs is None:
        print("  Mode 03: DTC read failed or was unsupported")
    elif not dtcs:
        print("  Mode 03: no stored DTCs")
    else:
        for code in dtcs:
            print(f"  [STORED]  {code}")
    return dtcs


def read_pending_dtcs(iface, channel, txid, rxid):
    """OBD-II Mode 07 – pending (not yet confirmed) DTCs."""
    resp = transact(iface, channel, txid, rxid, b'\x07', timeout_ms=2000)
    print(f"  Mode 07 raw: {hexs(resp) if resp else 'no response'}")
    dtcs = parse_dtc_response(resp, positive_sid=0x47)
    if dtcs is None:
        print("  Mode 07: DTC read failed or was unsupported")
    elif not dtcs:
        print("  Mode 07: no pending DTCs")
    else:
        for code in dtcs:
            print(f"  [PENDING]  {code}")
    return dtcs


def read_permanent_dtcs(iface, channel, txid, rxid):
    """OBD-II Mode 0A – permanent (non-erasable) DTCs."""
    resp = transact(iface, channel, txid, rxid, b'\x0A', timeout_ms=2000)
    print(f"  Mode 0A raw: {hexs(resp) if resp else 'no response'}")
    dtcs = parse_dtc_response(resp, positive_sid=0x4A)
    if dtcs is None:
        print("  Mode 0A: DTC read failed or was unsupported")
    elif not dtcs:
        print("  Mode 0A: no permanent DTCs")
    else:
        for code in dtcs:
            print(f"  [PERMANENT]  {code}")
    return dtcs


def read_obd_pid(iface, channel, txid, rxid, mode, pid, data_length, frame_number=0):
    """Read one live or freeze-frame PID; return (data, error description)."""
    request = bytes([mode, pid])
    prefix = bytes([mode + 0x40, pid])
    if mode == 0x02:
        request += bytes([frame_number])
        prefix += bytes([frame_number])
    resp = transact(iface, channel, txid, rxid, request, timeout_ms=800,
                    expected_prefix=prefix)
    if resp is None:
        return None, "no response (unsupported or unavailable)"
    if resp[0] == 0x7F:
        return None, f"negative response: {hexs(resp)}"
    if len(resp) < len(prefix) + data_length:
        return None, f"truncated response: {hexs(resp)}"
    return resp[len(prefix):len(prefix) + data_length], None


def print_diagnostic_measurements(iface, channel, txid, rxid, mode, frame_number=0):
    negative_responses = []
    for pid, label, size, decode, unit in DIAGNOSTIC_PIDS:
        data, error = read_obd_pid(iface, channel, txid, rxid, mode, pid, size,
                                   frame_number=frame_number)
        if error and error.startswith("negative response: "):
            negative_responses.append(error.removeprefix("negative response: "))
        elif error:
            print(f"    {label} (PID {pid:02X}): {error}")
        else:
            print(f"    {label} (PID {pid:02X}): {decode(data):.2f} {unit}")
    if negative_responses:
        responses = ', '.join(dict.fromkeys(negative_responses))
        print(f"    {len(negative_responses)} measurement(s) omitted due to "
              f"negative responses: {responses}")


def report_freeze_frames(iface, channel, txid, rxid, dtc_statuses):
    """Probe frame numbers and compare their associated DTCs with reported codes."""
    print(f"\n  Freeze frames 0-{FREEZE_FRAME_COUNT - 1}:")
    matched = set()
    associated = no_dtc = unavailable = 0
    for frame_number in range(FREEZE_FRAME_COUNT):
        data, error = read_obd_pid(iface, channel, txid, rxid, 0x02, 0x02, 2,
                                   frame_number=frame_number)
        if error:
            print(f"    Frame {frame_number}: {error}")
            unavailable += 1
        elif data == b'\x00\x00':
            print(f"    Frame {frame_number}: no associated DTC reported")
            no_dtc += 1
        else:
            frame_dtc = decode_dtc_pair(data[0], data[1])
            statuses = dtc_statuses.get(frame_dtc)
            status_text = ' | '.join(statuses) if statuses else 'not reported in DTC reads'
            print(f"    Frame {frame_number}: {frame_dtc} [{status_text}]")
            print("    Measurements (when this frame was recorded):")
            print_diagnostic_measurements(iface, channel, txid, rxid, 0x02,
                                          frame_number=frame_number)
            associated += 1
            matched.add(frame_dtc)

    print(f"  Frame probe results: {associated} with an associated DTC, "
          f"{no_dtc} with no DTC, {unavailable} unavailable or unconfirmed.")
    unmatched = [code for code in dtc_statuses if code not in matched]
    if unmatched:
        print(f"  No matching frame reported in this range for: {', '.join(unmatched)}")
    print("  Other frame numbers and manufacturer-specific snapshots were not checked.")


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
    incomplete = False

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
                incomplete = True
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

                if stored is None or pending is None or permanent is None:
                    incomplete = True
                stored = stored or []
                pending = pending or []
                permanent = permanent or []

                if stored:
                    all_stored[rxid]    = stored
                if pending:
                    all_pending[rxid]   = pending
                if permanent:
                    all_permanent[rxid] = permanent

                dtc_statuses = {}
                for status, codes in (("STORED", stored), ("PENDING", pending),
                                      ("PERMANENT", permanent)):
                    for code in codes:
                        dtc_statuses.setdefault(code, []).append(status)
                if dtc_statuses:
                    report_freeze_frames(iface, channel, txid, rxid, dtc_statuses)

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
        print("No DTCs found in successful responses.")
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

    if incomplete:
        print("Some DTC reads failed or were unsupported; results may be incomplete.")
    print("\nDone.")


if __name__ == '__main__':
    main()
