
# Diagnostic Trouble Code utilities for KIA (and Hyundai)

Glossary:
 * DTC: Diagnostic Trouble Code
 * OBD: On-Board Diagnostics

## Intro to OBD-II Diagnostic Trouble Codes (DTCs)

OBD-II DTCs are **not limited to one domain** — they span multiple systems. Each code follows the format: **1 letter + 4 digits** (e.g., `P0300`).

### Prefix / Domain

| Letter | Domain | Description |
|--------|--------|-------------|
| **P** | Powertrain | Engine, transmission, fuel system, emissions |
| **B** | Body | Airbags, climate control, seats, lighting |
| **C** | Chassis | ABS, traction control, steering, suspension |
| **U** | Network | CAN bus, module communication faults |

### Second digit (ownership)

| Digit | Meaning |
|-------|---------|
| **0** | SAE/ISO standard (generic, all OBD-II vehicles) |
| **1** | Manufacturer-specific |
| **2** | Manufacturer-specific |
| **3** | SAE/ISO reserved (or manufacturer-specific) |

### OBD-II Service Modes that retrieve DTCs

| Mode (PID) | Purpose |
|------------|---------|
| `0x03` | Read stored/confirmed DTCs |
| `0x07` | Read pending DTCs (detected but not yet confirmed) |
| `0x0A` | Read permanent DTCs (can't be cleared by a scan tool) |
| `0x04` | Clear stored DTCs + freeze frame |

### Important caveat

Standard OBD-II (ISO 15031 / SAE J1979) **only mandates Powertrain (P) codes** related to emissions. Reading **B, C, U codes** (like your airbag `B1346`) requires **manufacturer-specific protocols** — which is why your read_airbag_dtc.py uses a lower-level approach (likely via J2534 or a proprietary protocol) rather than a generic OBD-II mode.

In short: OBD-II mode `0x03` will only reliably return **emission-related P-codes**. Full system coverage (airbag, ABS, body, etc.) requires going beyond the standard.

## Goal

This set of tools allows communicating over the OBD port of KIA cars. Their primary goals are to retrieve OBD DTC:
 * `read_obd_dtc.py` - Read (P) codes with OBD II protocol
 * `read_ecu_dtc.py` - Firsct scan ECU presence and then read all codes directly from the found ECUs with UDS or KWP2000 protocols
 * `clear_obd_dtc.py` - Clear emission-related codes on discovered OBD-II ECUs

## Implementation

These tools rely on the Passthrough functionality (SAE J2534 API) of CAN probes: https://en.wikipedia.org/wiki/Passthrough_device

I have successfully tried with a KIA Rio 2012 and a Bosch MTS 6531.

The J2534 API of Bosch MTS 6531 is available thanks to the 32-bit DLL file `BVTX4J32.dll` that comes from Bosch VCI firmware: MTS_6531_VCI_firmware_setup_9.7.2922.175.exe

Install the VCI firmware setup file available on Bosch web site and the script will detect automatically the probe.

It is required to install Python for Windows (32-bit) since the Bosch VCI Firmware provides a 32-bit dll file. Please visit and download a python installer (>= 3.10) at https://www.python.org/downloads/windows/.

I am also using the great J2534 wrapper from https://github.com/Shamtam/PyJ2534.git that gets the DLL info from Windows registry.

## Procedure

Open a windows console (cmd.exe or PowerShell):
```bash
git clone https://github.com/angroyer/dtc-utils.git
cd dtc-utils
git clone https://github.com/angroyer/PyJ2534.git
```

Run one of the scripts.

### Read DTC from UDS or KWP2000 protocols

Run the script:
```bash
C:\Users\XXXXX\AppData\Local\Programs\Python\Python313-32\python.exe read_ecu_dtc.py
```

For reported UDS DTCs, including airbag codes, `read_ecu_dtc.py` requests
available snapshot records (service `19 03` and `19 04`) and extended data
(`19 06`) using the full three-byte DTC identifier. If record identification
is unavailable, it requests all snapshot records and falls back to records
`01`–`04`. Responses and failures are displayed for each request. The record
data is shown as raw hex because its layout and meaning can be manufacturer
specific; availability depends on the ECU and diagnostic session. These reads
do not clear DTCs. Airbag diagnosis and repairs should be handled by a
qualified technician.

Final summary of `read_ecu_dtc.py` will look as follows:
```bash
======================================================
  DTC SUMMARY
======================================================

  [KWP]  TX=0x7D1  RX=0x7D9  SRS / Airbag (primary HKMC address)
    (no DTCs)

  [KWP]  TX=0x7D2  RX=0x7DA  SRS / Airbag (alternate HKMC address)
    (no DTCs)

  [KWP]  TX=0x7D4  RX=0x7DC  SRS / Airbag (alternate address seen on some models)
    (no DTCs)

  [KWP]  TX=0x7E0  RX=0x7E8  ECM / PCM (Engine Control Module) [also OBD-II default]
    (no DTCs)

  [UDS]  TX=0x7D1  RX=0x7D9  SRS / Airbag (primary HKMC address)
    (no DTCs)

  [UDS]  TX=0x7D2  RX=0x7DA  SRS / Airbag (alternate HKMC address)
    B1102  status=0x08  [CONFIRMED]
    B1346  status=0x89  [ACTIVE | CONFIRMED | LAMP]

  [UDS]  TX=0x7D4  RX=0x7DC  SRS / Airbag (alternate address seen on some models)
    (no DTCs)

  [UDS]  TX=0x7E0  RX=0x7E8  ECM / PCM (Engine Control Module) [also OBD-II default]
    (no DTCs)

  Total unique DTCs across all ECUs: 2
======================================================
```

### Read DTC from OBD-II protocol

To read standard emission-related codes and freeze-frame data:
```bash
C:\Users\XXXXX\AppData\Local\Programs\Python\Python313-32\python.exe read_obd_dtc.py
```

For each ECU with stored, pending, or permanent codes, the script probes OBD-II
freeze-frame numbers 0 through 15 and displays selected measurements for each
frame with an associated DTC. It compares each associated DTC with the reported
code lists and notes codes with no matching frame in the checked range. Other
frame numbers and manufacturer-specific snapshots are not checked. An ECU may
not provide a separate freeze frame for every DTC; a negative response or timeout
does not prove a frame is empty. The scan may take time when frames do not respond.
PIDs and freeze frames depend on vehicle support and may be unavailable.
Measurement requests that receive negative responses are summarized rather than
displayed individually.
MAP is absolute pressure; these measurements do not identify a failed part.
Read them safely while parked, not while operating the computer and driving.
This report does not clear codes or freeze-frame data.

If the interface rejects the broadcast discovery filter, the scripts probe
individual OBD-II addresses using flow-control filters, including functional
requests. If no ECU responds, check the connection, ignition and vehicle support
for 11-bit OBD-II CAN at 500 kbps; the per-address messages may help diagnose it.

Example:
```bash
> C:\Users\XXXXX\AppData\Local\Programs\Python\Python313-32\python.exe read_obd_dtc.py
Using J2534 interface: 6531-Bosch
  DLL: C:\Program Files (x86)\Bosch\VTX-VCI\VCI Software (6531-Bosch)\Products\6531-Bosch\Dynamic Link Libraries\BVTX4J32.dll
Connected (ISO15765 @ 500 kbps)

Discovering ECUs via OBD-II functional request (Mode 01, PID 00)...
  Functional discovery filter unavailable: [ERR_INVALID_FILTER_ID] Invalid Filter ID value
  Falling back to individual slot probing...
  Found ECU: TX=0x7E0  RX=0x7E8  [ECM/PCM (engine)]
    No valid Mode 01 PID 00 response from RX=0x7E9.
    No valid Mode 01 PID 00 response from RX=0x7EA.
    No valid Mode 01 PID 00 response from RX=0x7EB.
    No valid Mode 01 PID 00 response from RX=0x7EC.
    No valid Mode 01 PID 00 response from RX=0x7ED.
    No valid Mode 01 PID 00 response from RX=0x7EE.
    No valid Mode 01 PID 00 response from RX=0x7EF.

==================================================
  ECU: ECM/PCM (engine)  (TX=0x7E0  RX=0x7E8)
==================================================
  VIN: UU1HSDJ9F54502991
  Mode 03 raw: 43 01 03 80
  [STORED]  P0380
  Mode 07 raw: 47 01 03 80
  [PENDING]  P0380
  Mode 0A raw: 4A 00
  Mode 0A: no permanent DTCs

  Freeze frames 0-15:
    Frame 0: P0380 [STORED | PENDING]
    Measurements (when this frame was recorded):
    Calculated engine load (PID 04): no response (unsupported or unavailable)
    Coolant temperature (PID 05): 22.00 C
    Intake manifold absolute pressure (MAP) (PID 0B): 10.00 kPa absolute
    Engine speed (PID 0C): 852.75 rpm
    Vehicle speed (PID 0D): 1.00 km/h
    Intake air temperature (PID 0F): 20.00 C
    Mass air flow (PID 10): 5.97 g/s
    Absolute throttle position (PID 11): 90.20 %
    Barometric pressure (PID 33): 100.00 kPa absolute
    Frame 1: no response (unsupported or unavailable)
    Frame 2: no response (unsupported or unavailable)
    Frame 3: no response (unsupported or unavailable)
    Frame 4: no response (unsupported or unavailable)
    Frame 5: no response (unsupported or unavailable)
    Frame 6: no response (unsupported or unavailable)
    Frame 7: no response (unsupported or unavailable)
    Frame 8: no response (unsupported or unavailable)
    Frame 9: no response (unsupported or unavailable)
    Frame 10: no response (unsupported or unavailable)
    Frame 11: no response (unsupported or unavailable)
    Frame 12: no response (unsupported or unavailable)
    Frame 13: no response (unsupported or unavailable)
    Frame 14: no response (unsupported or unavailable)
    Frame 15: no response (unsupported or unavailable)
  Frame probe results: 1 with an associated DTC, 0 with no DTC, 15 unavailable or unconfirmed.
  Other frame numbers and manufacturer-specific snapshots were not checked.

==================================================
SUMMARY
==================================================
  ECM/PCM (engine) – stored:    P0380
  ECM/PCM (engine) – pending:   P0380

Done.
````

### Clear DTC with OBD-II protocol

To clear emission-related codes on all discovered OBD-II ECUs:
```bash
C:\Users\XXXXX\AppData\Local\Programs\Python\Python313-32\python.exe clear_obd_dtc.py
```

The script lists discovered ECUs and requires `yes` before sending Mode 04 to
each one. Clearing may erase freeze-frame data and reset emissions readiness
monitors. Permanent DTCs cannot be cleared by a scan tool, and unresolved faults
may return. Discovery may miss ECUs; this does not clear UDS or KWP codes from
other systems, such as airbag or ABS. A timeout does not confirm whether clearing
occurred. Read and save diagnostic information before clearing.

Example
```bash
> C:\Users\XXXXX\AppData\Local\Programs\Python\Python313-32\python.exe clear_obd_dtc.py
Using J2534 interface: 6531-Bosch
  DLL: C:\Program Files (x86)\Bosch\VTX-VCI\VCI Software (6531-Bosch)\Products\6531-Bosch\Dynamic Link Libraries\BVTX4J32.dll
Connected (ISO15765 @ 500 kbps)

Discovering ECUs via OBD-II functional request (Mode 01, PID 00)...
  Functional discovery filter unavailable: [ERR_INVALID_FILTER_ID] Invalid Filter ID value
  Falling back to individual slot probing...
  Found ECU: TX=0x7E0  RX=0x7E8  [ECM/PCM (engine)]
    No valid Mode 01 PID 00 response from RX=0x7E9.
    No valid Mode 01 PID 00 response from RX=0x7EA.
    No valid Mode 01 PID 00 response from RX=0x7EB.
    No valid Mode 01 PID 00 response from RX=0x7EC.
    No valid Mode 01 PID 00 response from RX=0x7ED.
    No valid Mode 01 PID 00 response from RX=0x7EE.
    No valid Mode 01 PID 00 response from RX=0x7EF.

Mode 04 will request clearing on ALL discovered ECUs.
It can erase emissions DTCs and freeze frames and reset readiness monitors.
Permanent DTCs cannot be cleared by a scan tool; unresolved faults may return.
Discovery may miss ECUs; other diagnostic systems are not covered.
Type yes to continue: yes

ECU TX=0x7E0 RX=0x7E8:
  ECU accepted Mode 04; active faults may return.

All discovered ECUs accepted Mode 04.
```