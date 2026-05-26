"""Detect airbag (SRS) ECU on CAN/OBD via a J2534 interface (e.g. Bosch
MTS 6531 with BVTX4J32.dll), then read DTCs from it.

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

# 11-bit CAN ID pairs (request_id, response_id) for all known ECUs.
# Each entry: (request_id, response_id, description)
CANDIDATE_ECU_IDS = [
    # --- KIA RIO 2012 (UB, HKMC HS-CAN) ---
    (0x790, 0x798, "OBD functional (all ECUs)"),
    (0x793, 0x79B, "SMK (smart key unit)"),
    (0x7A3, 0x7AB, "PDM (passenger door module)"),
    (0x7A5, 0x7AD, "TPMS"),
    (0x7A6, 0x7AE, "TCU / telematics (UVO)"),
    (0x7A7, 0x7AF, "PGS / SPAS (parking guidance)"),
    (0x7B0, 0x7B8, "ABS / ESC (Anti-lock Brake / Electronic Stability Control)"),
    (0x7B2, 0x7BA, "ABS alternate"),
    (0x7B4, 0x7BC, "ESC alternate"),
    (0x7C2, 0x7CA, "BCM (Body Control Module)"),
    (0x7C3, 0x7CB, "BCM (alternate address seen on some models)"),
    (0x7C4, 0x7CC, "BCM (alternate address seen on some models)"),
    (0x7C5, 0x7CD, "BCM (alternate address seen on some models)"),
    (0x7C6, 0x7CE, "BCM (alternate address seen on some models)"),
    (0x7C7, 0x7CF, "BCM (alternate address seen on some models)"),
    (0x7C8, 0x7D0, "AVN (head unit)"),
    (0x7CB, 0x7D3, "external amplifier"),
    (0x7D1, 0x7D9, "SRS / Airbag (primary HKMC address)"),
    (0x7D2, 0x7DA, "SRS / Airbag (alternate HKMC address)"),
    (0x7D3, 0x7DB, "SRS / Airbag (alternate address seen on some models)"),
    (0x7D4, 0x7DC, "SRS / Airbag (alternate address seen on some models)"),
    (0x7D5, 0x7DD, "SRS / Airbag (alternate address seen on some models)"),
    (0x7D6, 0x7DE, "SRS / Airbag (alternate address seen on some models)"),
    (0x7D7, 0x7DF, "SRS / Airbag (alternate address seen on some models)"),
    (0x7E0, 0x7E8, "ECM / PCM (Engine Control Module) [also OBD-II default]"),
    (0x7E1, 0x7E9, "TCM (Automatic Transmission Control Module)"),
]


def hexs(data):
    return ' '.join(f'{b:02X}' for b in data)


def get_msg_bytes(msg):
    data = bytes(msg.Data or b'')
    if data:
        return data
    return bytes(msg.ExtraData or b'')


def build_iso15765_msg(txid, payload):
    """Build an outgoing ISO15765 PASSTHRU_MSG. The J2534 stack handles
    segmentation; we only supply CAN ID (4-byte BE) + UDS payload.
    """
    data = struct.pack('>I', txid) + bytes(payload)
    return PASSTHRU_MSG(
        ProtocolID.ISO15765,
        tx_flags=TxFlags.ISO15765_FRAME_PAD,
        data=data,
    )


def setup_flow_filter(iface, channel, txid, rxid):
    """Register a flow-control filter for the given (tx, rx) pair so
    the J2534 layer reassembles multi-frame responses from this ECU.
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


def request(iface, channel, txid, rxid, payload, timeout_ms=1000):
    """Send a UDS request and return the first non-echo, non-pending
    response payload (without the 2-byte CAN id), or None on timeout.
    """
    msg = build_iso15765_msg(txid, payload)
    iface.PassThruWriteMsgs(channel_id=channel, msgs=[msg], timeout=200)

    deadline = time.time() + timeout_ms / 1000.0
    while time.time() < deadline:
        try:
            responses = iface.PassThruReadMsgs(
                channel_id=channel, num_msgs=1, timeout=200,
            )
        except J2534Error:
            responses = []

        for r in responses:
            raw = get_msg_bytes(r)
            if len(raw) < 5:
                continue
            rid = struct.unpack('>I', raw[:4])[0]
            data = raw[4:]

            # Skip TX echoes / loopbacks.
            if int(r._RxStatus) & 0x09:  # TX_INDICATION | TX_MSG_TYPE
                continue
            if rid != rxid:
                continue

            # Handle UDS negative response 7F xx 78 (responsePending).
            if len(data) >= 3 and data[0] == 0x7F and data[2] == 0x78:
                deadline = time.time() + 5.0
                continue

            return data
    return None


def scan_ecus(iface, channel, probe_payload, label):
    """Probe every candidate (tx,rx) pair with the given payload.
    Returns a list of (txid, rxid, desc) triples that gave a valid response.
    """
    print(f"\nScanning with {label} ({hexs(probe_payload)})...")
    found = []
    for txid, rxid, desc in CANDIDATE_ECU_IDS:
        print(f"  probe TX={txid:03X} RX={rxid:03X}  [{desc}] ... ", end='', flush=True)
        try:
            fid = setup_flow_filter(iface, channel, txid, rxid)
        except J2534Error as e:
            print(f"filter error: {e}")
            continue

        try:
            resp = request(iface, channel, txid, rxid, probe_payload,
                           timeout_ms=400)
        finally:
            iface.PassThruStopMsgFilter(channel_id=channel,
                                        filter_id=fid)

        if resp is None:
            print("no reply")
            continue

        # Accept positive (0x50) or negative 0x7F <SID> — either means the
        # ECU is present and responded to our service request.
        if (resp[0] == 0x50 or
                (resp[0] == 0x7F and len(resp) >= 2
                 and resp[1] == probe_payload[0])):
            print(f"REPLY {hexs(resp)}  <-- ECU found")
            found.append((txid, rxid, desc))
        else:
            print(f"unexpected: {hexs(resp) if resp else 'none'}")

    return found


# --- DTC decoding (UDS 0x19 0x02 - ReportDTCByStatusMask) -----------

DTC_KIND = {0: 'P', 1: 'C', 2: 'B', 3: 'U'}


def decode_dtc(dtc_bytes):
    """Decode a 3-byte ISO 15031-6 / ISO 14229 DTC into "Bxxxx" form.
    Returns (code, failure_type): code is the 5-char DTC string, failure_type
    is the third byte (OBD failure type / snapshot identifier).
    """
    b1, b2, b3 = dtc_bytes
    kind = DTC_KIND[(b1 >> 6) & 0x03]
    digit2 = (b1 >> 4) & 0x03
    return f"{kind}{digit2}{(b1 & 0x0F):X}{b2:02X}", b3


def read_dtcs_uds(iface, channel, txid, rxid):
    """UDS 0x19 0x02 – try several status masks; 0xFF often gets no reply."""
    print("\nReading DTCs via UDS service 0x19 (ReadDTCInformation)...")
    resp = request(iface, channel, txid, rxid, b'\x10\x03', timeout_ms=500)  # extended session
    print(f"  raw response of session request: {hexs(resp) if resp else 'none'}")
    # Mask 0xFF is often rejected or gets no response; try 0x08 (confirmed)
    # first, then fall back to 0xFF and 0x89 (active|confirmed|lamp).
    resp = None
    for mask in (0x08, 0x89, 0xFF):
        resp = request(iface, channel, txid, rxid,
                       bytes([0x19, 0x02, mask]), timeout_ms=2000)
        if resp is not None and resp[0] != 0x7F:
            print(f"  (mask 0x{mask:02X} responded)")
            break
        resp = None  # try next mask
    if resp is None:
        print("  no response")
        return []
    if resp[0] == 0x7F:
        print(f"  negative response: {hexs(resp)}")
        return []
    if resp[0] != 0x59 or len(resp) < 3 or resp[1] != 0x02:
        print(f"  unexpected: {hexs(resp)}")
        return []

    # 0x59 0x02 <availabilityMask> [ <3-byte DTC> <statusByte> ]*
    avail = resp[2]
    records = resp[3:]
    dtcs = []
    print(f"  status availability mask: 0x{avail:02X}")
    if len(records) % 4 != 0:
        print(f"  warning: payload not a multiple of 4 bytes: {hexs(records)}")
    for i in range(0, len(records) - 3, 4):
        code, ftype = decode_dtc(records[i:i + 3])
        status = records[i + 3]
        dtcs.append((code, status))
        print(f"  DTC {code}  failureType=0x{ftype:02X}  status=0x{status:02X}")
    if not dtcs:
        print("  no DTC stored")
    return dtcs


# UDS DTC status-byte bit meanings (ISO 14229-1 Table D.1):
#   Bit 0 (0x01) testFailed                    – currently failing right now
#   Bit 1 (0x02) testFailedThisDriveCycle       – failed at least once this trip
#   Bit 2 (0x04) pendingDTC                     – detected but not yet confirmed
#   Bit 3 (0x08) confirmedDTC                   – confirmed / stored
#   Bit 4 (0x10) testNotCompletedSinceLastClear – monitor not yet run
#   Bit 5 (0x20) testFailedSinceLastClear       – HISTORICAL: failed since last clear
#   Bit 6 (0x40) testNotCompletedThisDriveCycle – monitor not run this trip
#   Bit 7 (0x80) warningIndicatorRequested      – this DTC is turning the lamp on
#
# To find "historical" faults that caused a lamp but are no longer active:
#   mask 0x08 – confirmed/stored
#   mask 0x20 – failed at some point since last clear (history)
#   mask 0x80 – DTC is currently requesting the warning lamp
#   mask 0xA9 – any of the above together with testFailed
#
# Sub-function 0x0A returns ALL DTCs the ECU *supports* regardless of status –
# the broadest possible query.

def _try_ascii(data):
    """Return printable ASCII representation of data bytes, or empty string."""
    try:
        s = data.decode('ascii')
        if all(0x20 <= ord(c) < 0x7F for c in s):
            return f'  → "{s}"'
    except Exception:
        pass
    return ''


def read_dtc_snapshot(iface, channel, txid, rxid, dtc_b1, dtc_b2, dtc_b3=0xFF):
    """UDS 0x19 0x04 / 0x19 0x06 – read freeze-frame and extended data
    for a single DTC identified by its raw 3-byte UDS DTC identifier.
    """
    code, _ = decode_dtc(bytes([dtc_b1, dtc_b2, dtc_b3]))
    # ── Freeze-frame (snapshot) ──────────────────────────────────────────────
    print(f"\n  Freeze-frame (0x19 0x04) for DTC {code}:")
    # Try record numbers 0x01-0x04 first; 0xFF (all records) is often rejected.
    ff_resp = None
    for rec_num in (0x01, 0x02, 0x03, 0x04, 0xFF):
        r = request(iface, channel, txid, rxid,
                    bytes([0x19, 0x04, dtc_b1, dtc_b2, dtc_b3, rec_num]),
                    timeout_ms=2000)
        if r is not None and r[0] != 0x7F:
            ff_resp = r
            print(f"    (record 0x{rec_num:02X} responded)")
            break
        if r is not None and r[0] == 0x7F:
            nrc = r[2] if len(r) >= 3 else 0
            if nrc not in (0x12, 0x31):  # show unexpected NRCs only
                print(f"    record 0x{rec_num:02X} negative: {hexs(r)}")
    resp = ff_resp
    if resp is None:
        print("    no freeze-frame data (ECU may not support this sub-function)")
    elif resp[0] == 0x7F:
        print(f"    negative: {hexs(resp)}")
    elif resp[0] == 0x59 and len(resp) >= 6 and resp[1] == 0x04:
        status = resp[5]
        flags = ' | '.join(filter(None, [
            'testFailed'     if status & 0x01 else '',
            'failedThisCycle' if status & 0x02 else '',
            'pending'        if status & 0x04 else '',
            'confirmed'      if status & 0x08 else '',
            'failedSinceClr' if status & 0x20 else '',
            'lamp'           if status & 0x80 else '',
        ]))
        print(f"    DTC status at snapshot: 0x{status:02X} [{flags}]")
        if len(resp) > 6:
            snap = resp[6:]
            print(f"    snapshot data: {hexs(snap)}")
            # Try to parse: [recNum numDIDs [DID_H DID_L data...]*]*
            idx = 0
            while idx + 1 < len(snap):
                rec_num  = snap[idx]
                num_dids = snap[idx + 1]
                idx += 2
                print(f"      record 0x{rec_num:02X}: {num_dids} DID(s)")
                for _ in range(num_dids):
                    if idx + 2 > len(snap):
                        break
                    did = (snap[idx] << 8) | snap[idx + 1]
                    idx += 2
                    # Collect remaining bytes as DID data (heuristic: stop
                    # when next 2 bytes look like a valid DID or end reached)
                    end = idx
                    while end + 2 <= len(snap):
                        next_did = (snap[end] << 8) | snap[end + 1]
                        if 0x0100 <= next_did <= 0xFFFF:
                            break
                        end += 1
                    did_data = snap[idx:end]
                    idx = end
                    print(f"        DID 0x{did:04X}: {hexs(did_data)}{_try_ascii(did_data)}")
                if num_dids == 0:
                    break
    else:
        print(f"    raw: {hexs(resp) if resp else 'none'}")

    # ── Extended data (0x19 0x06) ────────────────────────────────────────────
    print(f"  Extended data  (0x19 0x06) for DTC {code}:")
    resp = request(iface, channel, txid, rxid,
                   bytes([0x19, 0x06, dtc_b1, dtc_b2, dtc_b3, 0xFF]),
                   timeout_ms=3000)
    if resp is None:
        print("    no response")
    elif resp[0] == 0x7F:
        print(f"    negative: {hexs(resp)}")
    elif resp[0] == 0x59 and len(resp) >= 7 and resp[1] == 0x06:
        # resp: 59 06 b1 b2 b3 status [recNum data...]*
        ext = resp[6:]   # skip SID/SF + 3-byte DTC + status
        print(f"    raw ext: {hexs(ext)}")
        # Parse HKMC ext-data structure: [recNum(1) data...] repeating
        # For record 0xFF (all): expect recNum=FF then N data bytes
        idx = 0
        while idx < len(ext):
            rec = ext[idx]; idx += 1
            data = ext[idx:]
            print(f"    record 0x{rec:02X} ({len(data)} bytes): {hexs(data)}")
            # Decode known HKMC counters (little-endian 16-bit pairs)
            if len(data) >= 2:
                c1 = data[0] | (data[1] << 8)
                print(f"      [0-1] ignition-cycles since first detection : {c1}")
            if len(data) >= 4:
                c2 = data[2] | (data[3] << 8)
                print(f"      [2-3] ignition-cycles total (current)        : {c2}")
            if len(data) >= 5:
                print(f"      [4+]  remaining bytes: {hexs(data[4:])}")
            break  # only one record expected when recNum=0xFF
    else:
        print(f"    raw: {hexs(resp) if resp else 'none'}")


def probe_uds_dids(iface, channel, txid, rxid):
    """Try UDS service 0x22 (ReadDataByIdentifier) with known HKMC DIDs.
    Useful for ECUs that expose DTC data via DIDs rather than service 0x19.
    """
    print("\nProbing service 0x22 (ReadDataByIdentifier) for HKMC DIDs...")
    # Re-enter extended session so we probe in the right context.
    request(iface, channel, txid, rxid, b'\x10\x03', timeout_ms=500)
    dids = (
        # HKMC proprietary DTC/fault DIDs
        0xD001, 0xD002, 0xD003, 0xD010, 0xD011, 0xD020, 0xD021,
        0xD100, 0xD101, 0xD102,
        # ISO standardised identification DIDs (ECU info / part numbers)
        0xF101, 0xF102, 0xF103, 0xF110, 0xF111, 0xF112, 0xF113,
        0xF120, 0xF121, 0xF18A, 0xF18B, 0xF18C, 0xF18D, 0xF18E,
        0xF190, 0xF191, 0xF192, 0xF193, 0xF194, 0xF195, 0xF1A0,
        # Low-range HKMC DIDs
        0x0100, 0x0101, 0x0102, 0x0103, 0x0200, 0x0201, 0x0202,
    )
    found_any = False
    for did in dids:
        resp = request(iface, channel, txid, rxid,
                       bytes([0x22, (did >> 8) & 0xFF, did & 0xFF]),
                       timeout_ms=1000)
        if resp is None:
            continue   # skip silent no-response
        if resp[0] == 0x7F:
            nrc = resp[2] if len(resp) >= 3 else 0
            if nrc not in (0x11, 0x31):  # serviceNotSupported / requestOutOfRange
                print(f"  DID 0x{did:04X}: negative {hexs(resp)}")
            continue
        payload = resp[3:]   # strip 62 + 2-byte DID echo
        ascii_hint = _try_ascii(payload)
        line = f"  DID 0x{did:04X}: {hexs(payload)}{ascii_hint}"
        # For DID 0xF102 (ECU identification record), split into labeled fields.
        if did == 0xF102 and ascii_hint:
            fields = [f for f in ascii_hint.strip(' →"').strip('"').split() if f]
            if len(fields) >= 1:
                line += f"\n    Part number  : {fields[0]}"
            if len(fields) >= 2:
                line += f"\n    Cal SW ID    : {fields[1]}"
            if len(fields) >= 3:
                line += f"\n    HW version   : {fields[2]}"
        print(line)
        found_any = True
    if not found_any:
        print("  No UDS DIDs responded.")


def read_dtcs_uds_history(iface, channel, txid, rxid):
    """Supplement read_dtcs_uds() with additional UDS queries aimed at
    surfacing historical / confirmed DTCs that may be missed by mask 0xFF.

    Tries:
      0x19 0x02 with masks 0x08 (confirmed), 0x20 (failed since clear),
                            0x80 (lamp requested), 0x09 (failed|confirmed)
      0x19 0x0A  ReportSupportedDTCs  – every DTC the ECU can ever set
      0x19 0x0B  ReportFirstTestFailedDTC
      0x19 0x0C  ReportFirstConfirmedDTC
      0x19 0x0D  ReportMostRecentTestFailedDTC
      0x19 0x0E  ReportMostRecentConfirmedDTC
    """
    print("\nReading HISTORICAL DTCs via UDS 0x19 (extended queries)...")
    # Re-enter extended session so later queries don't get 0x7F (sessionTimeout).
    sresp = request(iface, channel, txid, rxid, b'\x10\x03', timeout_ms=500)
    if sresp is None or sresp[0] == 0x7F:
        # Try default session as fallback
        request(iface, channel, txid, rxid, b'\x10\x01', timeout_ms=500)

    # ── 1. Multiple status masks with sub-function 0x02 ─────────────────────
    MASKS = [
        (0x08, "confirmed (stored, bit3)"),
        (0x20, "failed since last clear (history, bit5)"),
        (0x80, "warning indicator requested = lamp on (bit7)"),
        (0x09, "testFailed | confirmed"),
        (0xA8, "confirmed | failedSinceClear | lampRequested"),
    ]
    # Maps DTC string code → (raw_b1, raw_b2, raw_b3, status)
    all_dtcs = {}
    for mask, label in MASKS:
        resp = request(iface, channel, txid, rxid,
                       bytes([0x19, 0x02, mask]), timeout_ms=2000)
        if resp is None or resp[0] == 0x7F:
            continue
        if resp[0] != 0x59 or len(resp) < 3 or resp[1] != 0x02:
            continue
        records = resp[3:]
        found = []
        for i in range(0, len(records) - 3, 4):
            b1, b2, b3 = records[i], records[i + 1], records[i + 2]
            code, _ = decode_dtc(bytes([b1, b2, b3]))
            status = records[i + 3]
            found.append((code, b1, b2, b3, status))
        if found:
            print(f"  mask 0x{mask:02X} ({label}):")
            for code, b1, b2, b3, status in found:
                status_bits = ' | '.join(filter(None, [
                    'ACTIVE'     if status & 0x01 else '',
                    'CONFIRMED'  if status & 0x08 else '',
                    'HISTORY'    if status & 0x20 else '',
                    'LAMP'       if status & 0x80 else '',
                ]))
                print(f"    DTC {code}  failureType=0x{b3:02X}  status=0x{status:02X}  [{status_bits}]")
                all_dtcs[code] = (b1, b2, b3, status)

    # ── 2. Sub-function 0x0A: ReportSupportedDTCs ────────────────────────────
    resp = request(iface, channel, txid, rxid, b'\x19\x0A', timeout_ms=2000)
    if resp and resp[0] == 0x59 and len(resp) >= 3 and resp[1] == 0x0A:
        records = resp[3:]
        print(f"  0x19 0x0A (all supported DTCs) – {len(records)//4} entries:")
        for i in range(0, len(records) - 3, 4):
            b1, b2, b3 = records[i], records[i + 1], records[i + 2]
            code, _ = decode_dtc(bytes([b1, b2, b3]))
            status = records[i + 3]
            print(f"    DTC {code}  status=0x{status:02X}")
            all_dtcs.setdefault(code, (b1, b2, b3, status))
    elif resp:
        print(f"  0x19 0x0A: {hexs(resp)}")

    # ── 3. Single-DTC sub-functions (0x0B–0x0E) ──────────────────────────────
    SF_LABELS = {
        0x0B: "first test-failed DTC",
        0x0C: "first confirmed DTC",
        0x0D: "most-recent test-failed DTC",
        0x0E: "most-recent confirmed DTC",
    }
    for sf, label in SF_LABELS.items():
        resp = request(iface, channel, txid, rxid,
                       bytes([0x19, sf]), timeout_ms=1000)
        if resp is None or resp[0] == 0x7F:
            continue
        if resp[0] != 0x59 or len(resp) < 3 or resp[1] != sf:
            continue
        if len(resp) < 6:
            print(f"  0x19 0x{sf:02X} ({label}): no DTC stored")
            continue
        b1, b2, b3 = resp[3], resp[4], resp[5]
        code, _ = decode_dtc(bytes([b1, b2, b3]))
        status = resp[6] if len(resp) > 6 else 0
        print(f"  0x19 0x{sf:02X} ({label}): {code}  status=0x{status:02X}")
        all_dtcs.setdefault(code, (b1, b2, b3, status))

    # ── 4. Freeze-frame for every unique DTC found ────────────────────────────
    if all_dtcs:
        request(iface, channel, txid, rxid, b'\x10\x03', timeout_ms=500)
        for code, (b1, b2, b3, _) in all_dtcs.items():
            read_dtc_snapshot(iface, channel, txid, rxid, b1, b2, b3)

    if not all_dtcs:
        print("  No historical DTCs found via extended queries.")
    return [(code, status) for code, (_, _, _, status) in all_dtcs.items()]


def read_dtcs_kwp_history(iface, channel, txid, rxid):
    """Try KWP2000 service 0x18 with additional status mask bytes to uncover
    historical / stored DTCs that the default mask=0x00 may not return.

    KWP2000 DTCStatusMask bits (ISO 14230 / HKMC convention):
      0x01 – DTC currently active / present
      0x02 – DTC previously stored (history / intermittent)
      0x04 – DTC stored but not yet confirmed (pending)
      0x08 – DTC stored in permanent memory
      0xFF – all
    """
    print("\nReading HISTORICAL DTCs via KWP2000 service 0x18 (extended masks)...")
    sresp = request(iface, channel, txid, rxid, b'\x10\x85', timeout_ms=500)
    print(f"  session response: {hexs(sresp) if sresp else 'none'}")

    MASKS = [
        (0xFF, "all bits"),
        (0x02, "history/intermittent (bit1)"),
        (0x04, "pending (bit2)"),
        (0x08, "permanent (bit3)"),
        (0x0F, "active | history | pending | permanent"),
    ]
    all_dtcs = {}
    for mask, label in MASKS:
        resp = request(iface, channel, txid, rxid,
                       bytes([0x18, mask, 0xFF, 0xFF]), timeout_ms=2000)
        if resp is None or resp[0] != 0x58:
            continue
        body = resp[1:]
        if body and body[0] == mask and len(body) % 3 != 0:
            body = body[1:]  # skip echoed mask byte
        if not body or len(body) % 3 != 0:
            continue
        found = []
        for i in range(0, len(body) - 2, 3):
            b1, b2, status = body[i], body[i + 1], body[i + 2]
            kind = DTC_KIND[(b1 >> 6) & 0x03]
            digit2 = (b1 >> 4) & 0x03
            code = f"{kind}{digit2}{(b1 & 0x0F):X}{b2:02X}"
            found.append((code, status))
        if found:
            print(f"  mask 0x{mask:02X} ({label}):")
            for code, status in found:
                print(f"    DTC {code}  status=0x{status:02X}")
                all_dtcs[code] = status

    # ── KWP2000 service 0x17 (ReadStatusOfDTC) ─────────────────────────────
    # Different from 0x18: returns DTC + status for a given group and mask.
    # Format: 17 <groupH> <groupL> <statusMask>
    # Response: 57 <groupH> <groupL> <mask> [DTC_H DTC_L DTCstatus]*
    print("  Trying KWP service 0x17 (ReadStatusOfDTC)...")
    for mask17 in (0xFF, 0x00, 0x04, 0x08):
        resp = request(iface, channel, txid, rxid,
                       bytes([0x17, 0xFF, 0xFF, mask17]), timeout_ms=2000)
        if resp is None or resp[0] != 0x57:
            continue
        body = resp[4:]   # skip 57 + 3 echo bytes
        if not body:
            continue
        if len(body) % 3 != 0:
            body = body[1:] if body[0] == mask17 else body  # skip echoed mask
        for i in range(0, len(body) - 2, 3):
            b1, b2, status = body[i], body[i + 1], body[i + 2]
            kind = DTC_KIND[(b1 >> 6) & 0x03]
            digit2 = (b1 >> 4) & 0x03
            code = f"{kind}{digit2}{(b1 & 0x0F):X}{b2:02X}"
            print(f"  0x17 mask=0x{mask17:02X}: DTC {code}  status=0x{status:02X}")
            all_dtcs[code] = status

    # ── HKMC proprietary services used by some MOBIS SRS ECUs ───────────────
    for svc, payloads, label in [
        (0xAA, (b'\xAA\x01', b'\xAA\x00', b'\xAA\x04'), "HKMC 0xAA"),
        (0xB6, (b'\xB6\x00\xFF\xFF', b'\xB6\x04\xFF\xFF'), "HKMC 0xB6"),
    ]:
        for payload in payloads:
            resp = request(iface, channel, txid, rxid, payload, timeout_ms=2000)
            if resp is None:
                continue
            pos_sid = svc + 0x40
            if resp[0] == 0x7F:
                print(f"  {label} {hexs(payload)}: negative {hexs(resp)}")
                continue
            print(f"  {label} {hexs(payload)}: {hexs(resp)}")
            if resp[0] == pos_sid:
                body = resp[1:]
                for i in range(0, len(body) - 2, 3):
                    b1, b2, status = body[i], body[i + 1], body[i + 2]
                    kind = DTC_KIND[(b1 >> 6) & 0x03]
                    digit2 = (b1 >> 4) & 0x03
                    code = f"{kind}{digit2}{(b1 & 0x0F):X}{b2:02X}"
                    print(f"    DTC {code}  status=0x{status:02X}")
                    all_dtcs[code] = status

    if not all_dtcs:
        print("  No historical DTCs found via KWP extended masks.")
    return list(all_dtcs.items())


def read_dtcs_kwp(iface, channel, txid, rxid):
    """KWP2000-on-CAN service 0x18 with mask=0x00 (HKMC: read all DTCs)."""
    print("\nReading DTCs via KWP2000 service 0x18 (ReadDTCByStatus)...")
    # Enter KWP extended diag session first.
    sresp = request(iface, channel, txid, rxid, b'\x10\x85', timeout_ms=500)
    # Enter KWP default session first.
    #sresp = request(iface, channel, txid, rxid, b'\x10\x81', timeout_ms=500)
    print(f"  session response: {hexs(sresp) if sresp else 'none'}")

    # Mask 0x00 is the only value this ECU accepts (HKMC-specific: means all).
    # Group 0xFF 0xFF = all DTC groups.
    resp = request(iface, channel, txid, rxid,
                   b'\x18\x00\xFF\xFF', timeout_ms=2000)
    print(f"  raw response: {hexs(resp) if resp else 'none'}")
    if resp is None:
        print("  no response")
        return []
    if resp[0] == 0x7F:
        print(f"  negative response: {hexs(resp)}")
        return []
    if resp[0] != 0x58:
        print(f"  unexpected: {hexs(resp)}")
        return []

    # Response: 58 <statusMask_echo> [DTC_H DTC_L DTCStatus]*
    # (no count byte in standard KWP2000 — records follow directly)
    dtcs = []
    body = resp[1:]  # skip the 0x58 positive SID
    # Some ECUs echo the mask as first byte; skip it if body[0] looks like
    # a mask (0x00) rather than a DTC high byte.
    if body and body[0] == 0x00 and len(body) % 3 != 0:
        body = body[1:]  # skip status mask echo
    if not body:
        print("  no DTC stored")
        return []
    if len(body) % 3 != 0:
        print(f"  warning: body length {len(body)} not multiple of 3: {hexs(body)}")
    for i in range(0, len(body) - 2, 3):
        b1, b2, status = body[i], body[i + 1], body[i + 2]
        kind = DTC_KIND[(b1 >> 6) & 0x03]
        digit2 = (b1 >> 4) & 0x03
        code = f"{kind}{digit2}{(b1 & 0x0F):X}{b2:02X}"
        dtcs.append((code, status))
        print(f"  DTC {code}  status=0x{status:02X}")
    if not dtcs:
        print("  no DTC stored")
    return dtcs


def read_dtcs_kwp13(iface, channel, txid, rxid):
    """KWP2000 service 0x13 (ReadDiagnosticTroubleCodes).
    Returns all currently stored DTCs without status filtering.
    Tries with a 2-byte group parameter (0xFF 0xFF = all groups) first,
    then without any parameter.
    """
    print("\nReading DTCs via KWP2000 service 0x13 (ReadDiagnosticTroubleCodes)...")
    sresp = request(iface, channel, txid, rxid, b'\x10\x85', timeout_ms=500)
    print(f"  session response: {hexs(sresp) if sresp else 'none'}")

    resp = None
    for payload in (b'\x13\xFF\xFF', b'\x13'):
        resp = request(iface, channel, txid, rxid, payload, timeout_ms=2000)
        if resp is not None:
            print(f"  raw response ({hexs(payload)}): {hexs(resp)}")
            break
    if resp is None:
        print("  no response")
        return []
    if resp[0] == 0x7F:
        print(f"  negative response: {hexs(resp)}")
        return []
    if resp[0] != 0x53:
        print(f"  unexpected: {hexs(resp)}")
        return []

    body = resp[1:]
    if not body:
        print("  no DTC stored")
        return []
    # Parse as count-prefixed [count DTC_H DTC_L ...] if consistent,
    # otherwise treat the whole body as raw DTC pairs.
    count = body[0]
    records = body[1:]
    if len(records) != count * 2:
        records = body  # no count byte present
    dtcs = []
    if len(records) % 2 != 0:
        print(f"  warning: odd payload length {len(records)}: {hexs(records)}")
    for i in range(0, len(records) - 1, 2):
        b1, b2 = records[i], records[i + 1]
        kind = DTC_KIND[(b1 >> 6) & 0x03]
        digit2 = (b1 >> 4) & 0x03
        code = f"{kind}{digit2}{(b1 & 0x0F):X}{b2:02X}"
        dtcs.append((code, None))
        print(f"  DTC {code}")
    if not dtcs:
        print("  no DTC stored")
    return dtcs


def decode_hkmc_srs_lid01(data, dtc_count):
    """Interpret the HKMC SRS LID 0x01 payload as N DTC records.

    LID 0x0A byte[0] gives the stored DTC count.  Each record is
    (len(data) // dtc_count) bytes with the following layout:
      Byte  0 : status/flags  (0x80 = warning-lamp active, 0x08 = confirmed,
                               0x20 = failed-since-clear, 0x01 = currently active)
      Bytes 1-2 : DTC code (HKMC proprietary 2-byte, SAE ISO 15031-6 encoded)
      Byte  3 : occurrence / repeat counter
      Bytes 4+ : snapshot / conditions at time of fault
    """
    if not data or dtc_count <= 0:
        return
    record_size = len(data) // dtc_count
    print(f"\n  === LID 0x01 decoded: {dtc_count} DTC record(s) x {record_size} bytes ===")
    for n in range(dtc_count):
        rec = data[n * record_size:(n + 1) * record_size]
        if not rec:
            break
        status = rec[0]
        flags = " | ".join(filter(None, [
            "LAMP"    if status & 0x80 else "",
            "HIST"    if status & 0x20 else "",
            "CONF"    if status & 0x08 else "",
            "PEND"    if status & 0x04 else "",
            "ACTIVE"  if status & 0x01 else "",
        ]))
        dtc_note = ""
        if len(rec) >= 3:
            b1, b2 = rec[1], rec[2]
            if not (b1 == 0x00 and b2 == 0x00) and b1 != 0xFF:
                kind = DTC_KIND[(b1 >> 6) & 0x03]
                d1 = (b1 >> 4) & 0x03
                d2 = b1 & 0x0F
                dtc_note = f"  DTC={kind}{d1}{d2:X}{b2:02X}"
        occur = f"  occur=0x{rec[3]:02X}" if len(rec) >= 4 else ""
        print(f"  Record {n}: {hexs(rec)}")
        print(f"    status=0x{status:02X} [{flags or 'none'}]{dtc_note}{occur}")


def probe_local_ids(iface, channel, txid, rxid):
    """Try KWP2000 service 0x21 (ReadDataByLocalIdentifier) for common
    HKMC SRS fault summary identifiers.
    """
    print("\nProbing service 0x21 (ReadDataByLocalIdentifier) for DTC data...")
    lids = (
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
        0x10, 0x20, 0x40,
        0x80, 0x81, 0x82, 0x83, 0x84,
        0x90, 0x91,
        0xC0, 0xC1, 0xD0, 0xD1, 0xFE, 0xFF,
    )
    lid01_data = None
    dtc_count  = 0
    for lid in lids:
        resp = request(iface, channel, txid, rxid,
                       bytes([0x21, lid]), timeout_ms=1000)
        if resp is None:
            print(f"  LID 0x{lid:02X}: no response")
        elif resp[0] == 0x7F:
            print(f"  LID 0x{lid:02X}: negative {hexs(resp)}")
        else:
            print(f"  LID 0x{lid:02X}: {hexs(resp)}")
            payload = resp[2:]   # strip positive SID + LID echo
            if lid == 0x01:
                lid01_data = payload
            elif lid == 0x0A and payload:
                # First byte = stored DTC count (HKMC convention)
                dtc_count = payload[0]
                print(f"    ^^^ LID 0x0A: {dtc_count} DTC(s) stored")
    if lid01_data is not None and dtc_count > 0:
        print("  NOTE: DTC codes below are decoded with ISO 15031-6 bit rules applied")
        print("  to a *proprietary* HKMC byte format.  Treat as approximate / indicative only.")
        decode_hkmc_srs_lid01(lid01_data, dtc_count)


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
        baud=500000,
    )
    print("Connected (ISO15765 @ 500 kbps)\n")

    # (proto, txid, rxid) -> {code: status}
    summary: dict[tuple, dict] = {}

    try:
        # ── KWP2000 pass ─────────────────────────────────────────────
        kwp_ecus = scan_ecus(iface, channel, b'\x10\x85', 'KWP2000')
        if not kwp_ecus:
            print("  No ECU responded to KWP2000 probe.")
        else:
            print(f"  KWP2000: {len(kwp_ecus)} ECU(s) found.")
            for txid, rxid, desc in kwp_ecus:
                print(f"\n==============================")
                print(f" [KWP]  TX=0x{txid:03X}  RX=0x{rxid:03X}")
                print(f" {desc}")
                print(f"==============================")
                ecu_key = ('KWP', txid, rxid, desc)
                summary[ecu_key] = {}
                fid = setup_flow_filter(iface, channel, txid, rxid)
                try:
                    dtcs = read_dtcs_kwp(iface, channel, txid, rxid)
                    if not dtcs:
                        dtcs = read_dtcs_kwp13(iface, channel, txid, rxid)
                    for code, status in dtcs:
                        summary[ecu_key][code] = status
                    # Always run history and LID probe – they may surface data
                    # even when the primary reads returned DTCs.
                    hist = read_dtcs_kwp_history(iface, channel, txid, rxid)
                    for code, status in hist:
                        summary[ecu_key].setdefault(code, status)
                    probe_local_ids(iface, channel, txid, rxid)
                finally:
                    iface.PassThruStopMsgFilter(channel_id=channel, filter_id=fid)

        # ── UDS pass ─────────────────────────────────────────────────
        uds_ecus = scan_ecus(iface, channel, b'\x10\x01', 'UDS')
        if not uds_ecus:
            print("  No ECU responded to UDS probe.")
        else:
            print(f"  UDS: {len(uds_ecus)} ECU(s) found.")
            for txid, rxid, desc in uds_ecus:
                print(f"\n==============================")
                print(f" [UDS]  TX=0x{txid:03X}  RX=0x{rxid:03X}")
                print(f" {desc}")
                print(f"==============================")
                ecu_key = ('UDS', txid, rxid, desc)
                summary[ecu_key] = {}
                fid = setup_flow_filter(iface, channel, txid, rxid)
                try:
                    request(iface, channel, txid, rxid, b'\x10\x01', timeout_ms=500)
                    dtcs = read_dtcs_uds(iface, channel, txid, rxid)
                    for code, status in dtcs:
                        summary[ecu_key][code] = status
                    # Always run all probes – each re-enters its own session.
                    hist = read_dtcs_uds_history(iface, channel, txid, rxid)
                    for code, status in hist:
                        summary[ecu_key].setdefault(code, status)
                    probe_uds_dids(iface, channel, txid, rxid)
                    probe_local_ids(iface, channel, txid, rxid)
                finally:
                    iface.PassThruStopMsgFilter(channel_id=channel, filter_id=fid)

    finally:
        iface.PassThruDisconnect(channel_id=channel)
        iface.PassThruClose(device_id=device_id)

        # ── DTC Summary ───────────────────────────────────────────────────────
        print("\n" + "=" * 54)
        print("  DTC SUMMARY")
        print("=" * 54)
        if not summary:
            print("  No ECUs responded – no DTCs collected.")
        else:
            total = 0
            for (proto, txid, rxid, desc), dtcs in summary.items():
                print(f"\n  [{proto}]  TX=0x{txid:03X}  RX=0x{rxid:03X}  {desc}")
                if not dtcs:
                    print("    (no DTCs)")
                else:
                    for code in sorted(dtcs):
                        status = dtcs[code]
                        if status is not None:
                            flags = ' | '.join(filter(None, [
                                'ACTIVE'    if status & 0x01 else '',
                                'CONFIRMED' if status & 0x08 else '',
                                'HISTORY'   if status & 0x20 else '',
                                'LAMP'      if status & 0x80 else '',
                            ]))
                            print(f"    {code}  status=0x{status:02X}  [{flags or '-'}]")
                        else:
                            print(f"    {code}")
                        total += 1
            print(f"\n  Total unique DTCs across all ECUs: {total}")
        print("=" * 54)
        print("\nDone.")


if __name__ == '__main__':
    main()
