"""Clear emission-related DTCs on discovered OBD-II CAN ECUs via J2534."""

import sys

from PyJ2534.PyJ2534 import get_interfaces, load_interface, ProtocolID
from PyJ2534.PyJ2534.error import J2534Error

from read_obd_dtc import discover_ecus, hexs, setup_flow_filter, transact


def clear_ecus(iface, channel, ecus):
    """Send Mode 04 to each discovered ECU and report its response."""
    failed = False
    for txid, rxid in ecus:
        print(f"\nECU TX=0x{txid:03X} RX=0x{rxid:03X}:")
        try:
            fid = setup_flow_filter(iface, channel, txid, rxid)
        except J2534Error as e:
            print(f"  Could not set flow-control filter: {e}")
            failed = True
            continue

        try:
            try:
                resp = transact(iface, channel, txid, rxid, b'\x04',
                                timeout_ms=3000, expected_prefix=b'\x44')
                if resp is None:
                    print("  No response; clearing outcome is unknown.")
                    failed = True
                elif resp[0] == 0x44:
                    print("  ECU accepted Mode 04; active faults may return.")
                else:
                    print(f"  Mode 04 rejected: {hexs(resp)}")
                    failed = True
            except J2534Error as e:
                print(f"  Mode 04 communication error; outcome may be unknown: {e}")
                failed = True
        finally:
            try:
                iface.PassThruStopMsgFilter(channel_id=channel, filter_id=fid)
            except J2534Error as e:
                print(f"  Could not stop flow-control filter: {e}")
                failed = True

    if failed:
        print("\nSome ECU clearing requests failed or could not be confirmed.")
        return 1
    print("\nAll discovered ECUs accepted Mode 04.")
    return 0


def main():
    ifaces = get_interfaces()
    if not ifaces:
        print("No J2534 interface found in registry.")
        return 1

    name, path = next(iter(ifaces.items()))
    print(f"Using J2534 interface: {name}")
    print(f"  DLL: {path}")
    iface = load_interface(path)
    device_id = iface.PassThruOpen()
    try:
        channel = iface.PassThruConnect(
            device_id=device_id,
            protocol=ProtocolID.ISO15765,
            flags=0,
            baud=500000,
        )
        try:
            print("Connected (ISO15765 @ 500 kbps)")
            ecus = discover_ecus(iface, channel)
            if not ecus:
                print("No OBD-II ECUs discovered; nothing was cleared.")
                return 1

            print("\nMode 04 will request clearing on ALL discovered ECUs.")
            print("It can erase emissions DTCs and freeze frames and reset readiness monitors.")
            print("Permanent DTCs cannot be cleared by a scan tool; unresolved faults may return.")
            print("Discovery may miss ECUs; other diagnostic systems are not covered.")
            try:
                confirmation = input("Type yes to continue: ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                print("\nCancelled; nothing was cleared.")
                return 0
            if confirmation != 'yes':
                print("Cancelled; nothing was cleared.")
                return 0
            return clear_ecus(iface, channel, ecus)
        finally:
            iface.PassThruDisconnect(channel_id=channel)
    finally:
        iface.PassThruClose(device_id=device_id)


if __name__ == '__main__':
    sys.exit(main())
