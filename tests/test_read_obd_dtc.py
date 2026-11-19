import contextlib
import io
import struct
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import read_obd_dtc as obd
from PyJ2534.PyJ2534.error import J2534Error, J2534Errors


class FreezeFrameTests(unittest.TestCase):
    def setUp(self):
        frame_count = patch.object(obd, 'FREEZE_FRAME_COUNT', 1)
        frame_count.start()
        self.addCleanup(frame_count.stop)

    def test_matching_freeze_frame_values(self):
        requests = []

        def transact(iface, channel, txid, rxid, payload, **kwargs):
            requests.append(payload)
            mode, pid = payload[:2]
            prefix = bytes([mode + 0x40, pid])
            if mode == 2:
                prefix += payload[2:3]
            if mode == 2 and pid == 2:
                return prefix + b'\x02\x99'
            values = {
                0x04: b'\x80', 0x05: b'\x5A', 0x0B: b'\x78',
                0x0C: b'\x1F\x40', 0x0D: b'\x50', 0x0F: b'\x3C',
                0x10: b'\x04\xD2', 0x11: b'\xFF', 0x33: b'\x64',
            }
            return prefix + values[pid]

        output = io.StringIO()
        with patch.object(obd, 'transact', side_effect=transact), contextlib.redirect_stdout(output):
            obd.report_freeze_frames(None, 1, 0x7E0, 0x7E8, {'P0299': ['STORED']})
        text = output.getvalue()
        self.assertIn("Frame 0: P0299 [STORED]", text)
        self.assertIn("Engine speed (PID 0C): 2000.00 rpm", text)
        self.assertIn("Mass air flow (PID 10): 12.34 g/s", text)
        self.assertIn("Intake manifold absolute pressure (MAP) (PID 0B): 120.00 kPa absolute", text)
        self.assertIn("Barometric pressure (PID 33): 100.00 kPa absolute", text)
        self.assertNotIn("Current measurements", text)
        self.assertIn(b'\x02\x0C\x00', requests)
        self.assertNotIn(b'\x01\x0C', requests)

    def test_negative_measurements_are_omitted_but_other_errors_remain(self):
        def transact(iface, channel, txid, rxid, payload, **kwargs):
            if payload == b'\x02\x02\x00':
                return b'\x42\x02\x00\x13\x51'
            if payload in (b'\x02\x0F\x00', b'\x02\x33\x00'):
                return b'\x7F\x02\x12'
            if payload == b'\x02\x0C\x00':
                return b'\x42\x0C\x00\x01'
            return None

        output = io.StringIO()
        with patch.object(obd, 'transact', side_effect=transact), contextlib.redirect_stdout(output):
            obd.report_freeze_frames(None, 1, 0x7E0, 0x7E8, {'P1351': ['STORED']})
        text = output.getvalue()
        self.assertNotIn("Intake air temperature (PID 0F)", text)
        self.assertNotIn("Barometric pressure (PID 33)", text)
        self.assertIn("2 measurement(s) omitted due to negative responses: 7F 02 12", text)
        self.assertIn("Engine speed (PID 0C): truncated response", text)
        self.assertIn("Calculated engine load (PID 04): no response", text)

    def test_other_freeze_frame_is_shown_for_its_associated_dtc(self):
        requests = []

        def transact(iface, channel, txid, rxid, payload, **kwargs):
            requests.append(payload)
            if payload == b'\x02\x02\x00':
                return b'\x42\x02\x00\x03\x00'
            if payload == b'\x02\x0C\x00':
                return b'\x42\x0C\x00\x0B\xF4'
            return None

        output = io.StringIO()
        with patch.object(obd, 'transact', side_effect=transact), contextlib.redirect_stdout(output):
            obd.report_freeze_frames(None, 1, 0x7E0, 0x7E8,
                                     {'P0299': ['PENDING'], 'P0300': ['STORED']})
        self.assertIn("Frame 0: P0300 [STORED]", output.getvalue())
        self.assertIn("Engine speed (PID 0C): 765.00 rpm", output.getvalue())
        self.assertIn("No matching frame reported in this range for: P0299", output.getvalue())
        self.assertIn("no response", output.getvalue())
        self.assertEqual(requests.count(b'\x02\x02\x00'), 1)
        self.assertFalse(any(request[0] == 1 for request in requests))

    def test_unlisted_freeze_frame_dtc_is_identified(self):
        def transact(iface, channel, txid, rxid, payload, **kwargs):
            if payload == b'\x02\x02\x00':
                return b'\x42\x02\x00\x13\x51'
            return None

        output = io.StringIO()
        with patch.object(obd, 'transact', side_effect=transact), contextlib.redirect_stdout(output):
            obd.report_freeze_frames(None, 1, 0x7E0, 0x7E8, {'P0299': ['STORED']})
        self.assertIn("Frame 0: P1351 [not reported in DTC reads]", output.getvalue())
        self.assertIn("No matching frame reported in this range for: P0299", output.getvalue())

    def test_empty_freeze_frame_is_not_attributed_to_p0299(self):
        requests = []

        def transact(iface, channel, txid, rxid, payload, **kwargs):
            requests.append(payload)
            if payload == b'\x02\x02\x00':
                return b'\x42\x02\x00\x00\x00'
            return None

        output = io.StringIO()
        with patch.object(obd, 'transact', side_effect=transact), contextlib.redirect_stdout(output):
            obd.report_freeze_frames(None, 1, 0x7E0, 0x7E8, {'P0299': ['STORED']})
        self.assertIn("no associated DTC reported", output.getvalue())
        self.assertFalse(any(request[0] == 2 and request[1] != 2 for request in requests))

    def test_unavailable_frame_does_not_query_measurements(self):
        output = io.StringIO()
        with (patch.object(obd, 'transact', return_value=b'\x7F\x02\x12') as transact,
              contextlib.redirect_stdout(output)):
            obd.report_freeze_frames(None, 1, 0x7E0, 0x7E8, {'P1351': ['STORED']})
        self.assertIn("negative response: 7F 02 12", output.getvalue())
        self.assertEqual(transact.call_count, 1)

    def test_scans_all_sixteen_frames_and_matches_pending_codes(self):
        requests = []

        def transact(iface, channel, txid, rxid, payload, **kwargs):
            requests.append(payload)
            pid, frame = payload[1], payload[2]
            if pid == 2:
                dtcs = {0: b'\x13\x51', 1: b'\x00\x00',
                        7: b'\x02\x99', 15: b'\x03\x00'}
                if frame in dtcs:
                    return b'\x42\x02' + bytes([frame]) + dtcs[frame]
                if frame == 3:
                    return None
                if frame == 4:
                    return b'\x42\x02\x04\x02'
                return b'\x7F\x02\x12'
            if pid == 0x0C:
                return b'\x42\x0C' + bytes([frame, 0x0B, 0xF4])
            return b'\x7F\x02\x12'

        output = io.StringIO()
        with (patch.object(obd, 'FREEZE_FRAME_COUNT', 16),
              patch.object(obd, 'transact', side_effect=transact),
              contextlib.redirect_stdout(output)):
            obd.report_freeze_frames(None, 1, 0x7E0, 0x7E8, {
                'P1351': ['STORED', 'PENDING'],
                'P0299': ['PENDING'],
                'P0111': ['PERMANENT'],
            })
        text = output.getvalue()
        self.assertIn("Frame 0: P1351 [STORED | PENDING]", text)
        self.assertIn("Frame 7: P0299 [PENDING]", text)
        self.assertIn("Frame 15: P0300 [not reported in DTC reads]", text)
        self.assertIn("Frame 1: no associated DTC reported", text)
        self.assertIn("Frame 3: no response", text)
        self.assertIn("Frame 4: truncated response", text)
        self.assertIn("3 with an associated DTC, 1 with no DTC, 12 unavailable", text)
        self.assertIn("No matching frame reported in this range for: P0111", text)
        self.assertEqual([request[2] for request in requests if request[1] == 2], list(range(16)))
        self.assertIn(b'\x02\x0C\x07', requests)
        self.assertIn(b'\x02\x0C\x0F', requests)
        self.assertNotIn(b'\x02\x0C\x01', requests)

    def test_pid_negative_and_truncated_responses_are_not_values(self):
        with patch.object(obd, 'transact', return_value=b'\x7F\x02\x12'):
            data, error = obd.read_obd_pid(None, 1, 0x7E0, 0x7E8, 2, 2, 2)
        self.assertIsNone(data)
        self.assertIn("negative response: 7F 02 12", error)
        with patch.object(obd, 'transact', return_value=b'\x42\x02\x00\x02'):
            data, error = obd.read_obd_pid(None, 1, 0x7E0, 0x7E8, 2, 2, 2)
        self.assertIsNone(data)
        self.assertIn("truncated response", error)

    def test_main_reports_frame_once_per_ecu_including_permanent_only(self):
        iface = Mock()
        with (patch.object(obd, 'get_interfaces', return_value={'adapter': 'dll'}),
              patch.object(obd, 'load_interface', return_value=iface),
              patch.object(obd, 'discover_ecus', return_value=[
                  (0x7E0, 0x7E8), (0x7E1, 0x7E9), (0x7E2, 0x7EA)]),
              patch.object(obd, 'setup_flow_filter', return_value=1),
              patch.object(obd, 'read_vin', return_value=None),
              patch.object(obd, 'read_stored_dtcs', side_effect=[['P0299', 'P1351'], [], []]),
              patch.object(obd, 'read_pending_dtcs', side_effect=[['P0299'], [], []]),
              patch.object(obd, 'read_permanent_dtcs', side_effect=[['P1351'], ['P0300'], []]),
              patch.object(obd, 'report_freeze_frames') as freeze,
              contextlib.redirect_stdout(io.StringIO())):
            obd.main()
        self.assertEqual(freeze.call_count, 2)
        freeze.assert_any_call(iface, iface.PassThruConnect.return_value,
                               0x7E0, 0x7E8,
                               {'P0299': ['STORED', 'PENDING'],
                                'P1351': ['STORED', 'PERMANENT']})
        freeze.assert_any_call(iface, iface.PassThruConnect.return_value,
                               0x7E1, 0x7E9, {'P0300': ['PERMANENT']})

    def test_pending_only_code_triggers_frame_probe(self):
        iface = Mock()
        with (patch.object(obd, 'get_interfaces', return_value={'adapter': 'dll'}),
              patch.object(obd, 'load_interface', return_value=iface),
              patch.object(obd, 'discover_ecus', return_value=[(0x7E0, 0x7E8)]),
              patch.object(obd, 'setup_flow_filter', return_value=1),
              patch.object(obd, 'read_vin', return_value=None),
              patch.object(obd, 'read_stored_dtcs', return_value=[]),
              patch.object(obd, 'read_pending_dtcs', return_value=['P0299']),
              patch.object(obd, 'read_permanent_dtcs', return_value=[]),
              patch.object(obd, 'report_freeze_frames') as freeze,
              contextlib.redirect_stdout(io.StringIO())):
            obd.main()
        freeze.assert_called_once_with(iface, iface.PassThruConnect.return_value,
                                       0x7E0, 0x7E8, {'P0299': ['PENDING']})

    def test_negative_permanent_response_is_not_reported_as_no_codes(self):
        output = io.StringIO()
        with patch.object(obd, 'transact', return_value=b'\x7F\x0A\x11'), contextlib.redirect_stdout(output):
            dtcs = obd.read_permanent_dtcs(None, 1, 0x7E0, 0x7E8)
        self.assertIsNone(dtcs)
        self.assertIn("DTC read failed or was unsupported", output.getvalue())
        self.assertNotIn("no permanent DTCs", output.getvalue())

    def test_summary_warns_when_dtc_results_are_incomplete(self):
        iface = Mock()
        output = io.StringIO()
        with (patch.object(obd, 'get_interfaces', return_value={'adapter': 'dll'}),
              patch.object(obd, 'load_interface', return_value=iface),
              patch.object(obd, 'discover_ecus', return_value=[(0x7E0, 0x7E8)]),
              patch.object(obd, 'setup_flow_filter', return_value=1),
              patch.object(obd, 'read_vin', return_value=None),
              patch.object(obd, 'read_stored_dtcs', return_value=[]),
              patch.object(obd, 'read_pending_dtcs', return_value=[]),
              patch.object(obd, 'read_permanent_dtcs', return_value=None),
              contextlib.redirect_stdout(output)):
            obd.main()
        self.assertIn("results may be incomplete", output.getvalue())
        self.assertIn("No DTCs found in successful responses", output.getvalue())


class TransactionTests(unittest.TestCase):
    def test_unrelated_response_is_not_used_for_requested_pid(self):
        def message(payload):
            return SimpleNamespace(
                Data=struct.pack('>I', 0x7E8) + payload,
                ExtraData=b'',
                _RxStatus=0,
            )

        iface = Mock()
        iface.PassThruReadMsgs.side_effect = [
            [message(b'\x41\x0C\x00\x00')],
            [message(b'\x41\x0B\x78')],
        ]
        data, error = obd.read_obd_pid(iface, 1, 0x7E0, 0x7E8, 1, 0x0B, 1)
        self.assertEqual(data, b'\x78')
        self.assertIsNone(error)

    def test_j2534_read_failure_is_not_reported_as_missing_pid(self):
        iface = Mock()
        iface.PassThruReadMsgs.side_effect = J2534Error(J2534Errors.ERR_FAILED)
        with self.assertRaises(J2534Error):
            obd.read_obd_pid(iface, 1, 0x7E0, 0x7E8, 1, 0x0B, 1)

    def test_response_from_another_frame_is_not_used(self):
        def message(payload):
            return SimpleNamespace(
                Data=struct.pack('>I', 0x7E8) + payload,
                ExtraData=b'',
                _RxStatus=0,
            )

        iface = Mock()
        iface.PassThruReadMsgs.side_effect = [
            [message(b'\x42\x0C\x00\x00\x00')],
            [message(b'\x42\x0C\x07\x0B\xF4')],
        ]
        data, error = obd.read_obd_pid(iface, 1, 0x7E0, 0x7E8, 2, 0x0C, 2,
                                       frame_number=7)
        self.assertEqual(data, b'\x0B\xF4')
        self.assertIsNone(error)


if __name__ == '__main__':
    unittest.main()
