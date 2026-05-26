
# Data Trouble Code utilities for KIA (and Hyundai)

Glossary:
 * DTC: Data Trouble Code
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

Run the script:
```bash
C:\Users\XXXXX\AppData\Local\Programs\Python\Python313-32\python.exe read_ecu_dtc.py
```
