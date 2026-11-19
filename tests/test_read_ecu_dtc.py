import contextlib
import io
import struct
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import read_ecu_dtc as ecu
from PyJ2534.PyJ2534.error import J2534Error, J2534Errors


class UdsAdditionalDataTests(unittest.TestCase):
    def test_primary_read_retains_full_dtc_identifier(self):
        with (patch.object(ecu, 'request', side_effect=[
                b'\x50\x03', b'\x59\x02\xFF\x93\x46\x12\x89']),
              contextlib.redirect_stdout(io.StringIO())):
            dtcs = ecu.read_dtcs_uds(None, 1, 0x7D2, 0x7DA)
        self.assertEqual(dtcs, [('B1346', 0x89, b'\x93\x46\x12')])

    def test_identification_uses_all_three_dtc_bytes_and_record_numbers(self):
        response = b'\x59\x03\x93\x46\x01\x02\x93\x46\x02\x09'
        with patch.object(ecu, 'request', return_value=response):
            records = ecu.read_snapshot_identifiers(None, 1, 0x7D2, 0x7DA)
        self.assertEqual(records, {b'\x93\x46\x01': [2], b'\x93\x46\x02': [9]})

    def test_malformed_snapshot_identification_is_reported(self):
        output = io.StringIO()
        with (patch.object(ecu, 'request', return_value=b'\x59\x03\x93'),
              contextlib.redirect_stdout(output)):
            records = ecu.read_snapshot_identifiers(None, 1, 0x7D2, 0x7DA)
        self.assertEqual(records, {})
        self.assertIn('malformed: 59 03 93', output.getvalue())

    def test_discovered_records_are_all_read_without_guessing_data(self):
        dtc = b'\x93\x46\x12'
        calls = []

        def fake_request(iface, channel, txid, rxid, payload, **kwargs):
            calls.append(payload)
            if payload[1] == 4:
                return b'\x59\x04' + dtc + b'\x89' + bytes([payload[-1], 1, 0xF1, 0x90, 0x12, 0x34])
            return b'\x59\x06' + dtc + b'\x89\x01\x34\x12\x78\x56'

        output = io.StringIO()
        with (patch.object(ecu, 'request', side_effect=fake_request),
              contextlib.redirect_stdout(output)):
            ecu.read_dtc_snapshot(None, 1, 0x7D2, 0x7DA, dtc, [2, 9])
        self.assertEqual(calls, [
            b'\x19\x04\x93\x46\x12\x02',
            b'\x19\x04\x93\x46\x12\x09',
            b'\x19\x06\x93\x46\x12\xFF',
        ])
        text = output.getvalue()
        self.assertIn('B1346 (full DTC 93 46 12, failure type 0x12)', text)
        self.assertIn('request record 0x09', text)
        self.assertIn('raw record data: 02 01 F1 90 12 34', text)
        self.assertIn('raw record data: 01 34 12 78 56', text)
        self.assertNotIn('ignition-cycles', text)
        self.assertNotIn('DTC status at snapshot', text)

    def test_fallback_requests_all_then_each_record(self):
        dtc = b'\x91\x02\x00'
        calls = []

        def fake_request(iface, channel, txid, rxid, payload, **kwargs):
            calls.append(payload)
            if payload[1] == 4 and payload[-1] in (1, 3):
                return b'\x59\x04' + dtc + b'\x08' + bytes([payload[-1], 0])
            return b'\x7F\x19\x31'

        output = io.StringIO()
        with (patch.object(ecu, 'request', side_effect=fake_request),
              contextlib.redirect_stdout(output)):
            ecu.read_dtc_snapshot(None, 1, 0x7D2, 0x7DA, dtc, [])
        self.assertEqual([payload[-1] for payload in calls if payload[1] == 4],
                         [0xFF, 1, 2, 3, 4])
        self.assertIn('negative response 7F 19 31', output.getvalue())
        self.assertIn('raw record data: 03 00', output.getvalue())

    def test_incomplete_or_wrong_dtc_and_record_not_attributed(self):
        dtc = b'\x93\x46\x12'
        responses = [
            b'\x59\x04\x93\x46',
            b'\x59\x04\x93\x46\x13\x89\x01\x00',
            b'\x59\x04\x93\x46\x12\x89\x01',
            b'\x59\x04\x93\x46\x12\x89\x02\x00',
        ]
        output = io.StringIO()
        with (patch.object(ecu, 'request', side_effect=responses),
              contextlib.redirect_stdout(output)):
            for _ in responses:
                self.assertFalse(ecu.read_dtc_data_record(
                    None, 1, 0x7D2, 0x7DA, dtc, 4, 1))
        self.assertEqual(output.getvalue().count('malformed or mismatched'), 2)
        self.assertIn('truncated snapshot record', output.getvalue())
        self.assertIn('mismatched snapshot record', output.getvalue())
        self.assertNotIn('raw record data:', output.getvalue())

    def test_history_includes_primary_only_dtc_and_separate_failure_types(self):
        primary = [('B1346', 0x89, b'\x93\x46\x01'),
                   ('B1346', 0x08, b'\x93\x46\x02')]

        def fake_request(iface, channel, txid, rxid, payload, **kwargs):
            if payload == b'\x19\x03':
                return b'\x59\x03\x93\x46\x02\x07'
            return b'\x7F' + payload[:1] + b'\x11'

        with (patch.object(ecu, 'request', side_effect=fake_request),
              patch.object(ecu, 'read_dtc_snapshot') as snapshot,
              contextlib.redirect_stdout(io.StringIO())):
            ecu.read_dtcs_uds_history(None, 1, 0x7D2, 0x7DA, primary)
        self.assertEqual(snapshot.call_count, 2)
        snapshot.assert_any_call(None, 1, 0x7D2, 0x7DA, b'\x93\x46\x01', [])
        snapshot.assert_any_call(None, 1, 0x7D2, 0x7DA, b'\x93\x46\x02', [7])

    def test_wrong_subfunction_response_is_ignored(self):
        def message(payload):
            return SimpleNamespace(Data=struct.pack('>I', 0x7DA) + payload,
                                   ExtraData=b'', _RxStatus=0)

        iface = Mock()
        iface.PassThruReadMsgs.side_effect = [
            [message(b'\x59\x03')],
            [message(b'\x59\x04\x93\x46\x12\x89\x01\x00')],
        ]
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            ok = ecu.read_dtc_data_record(iface, 1, 0x7D2, 0x7DA,
                                          b'\x93\x46\x12', 4, 1)
        self.assertTrue(ok)
        self.assertIn('raw record data: 01 00', output.getvalue())

    def test_j2534_read_failure_is_not_swallowed(self):
        iface = Mock()
        iface.PassThruReadMsgs.side_effect = J2534Error(J2534Errors.ERR_FAILED)
        with self.assertRaises(J2534Error):
            ecu.read_snapshot_identifiers(iface, 1, 0x7D2, 0x7DA)


if __name__ == '__main__':
    unittest.main()
