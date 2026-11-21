import contextlib
import io
import struct
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, call, patch

import clear_obd_dtc as clear
import read_obd_dtc as obd
from PyJ2534.PyJ2534.error import J2534Error, J2534Errors


class DiscoveryTests(unittest.TestCase):
    def test_functional_discovery_accepts_all_slots_and_matching_responses(self):
        def message(rxid, payload):
            return SimpleNamespace(
                Data=struct.pack('>I', rxid) + payload,
                ExtraData=b'',
                _RxStatus=0,
            )

        iface = Mock()
        iface.PassThruReadMsgs.side_effect = [
            [message(0x7E8, b'\x41\x00\x00\x00\x00\x01')],
            [message(0x7E9, b'\x41\x00\x00\x00\x00\x01')],
            [message(0x7EA, b'\x41\x0C\x00\x00\x00\x00')],
            [message(0x7EB, b'\x41\x00')],
        ]
        with (patch.object(obd.time, 'time', side_effect=[0, 0, 0, 0, 0, 2]),
              contextlib.redirect_stdout(io.StringIO())):
            self.assertEqual(obd.discover_ecus(iface, 1),
                             [(0x7E0, 0x7E8), (0x7E1, 0x7E9)])
        mask = iface.PassThruStartMsgFilter.call_args.kwargs['mask_msg']
        self.assertEqual(mask.Data, b'\xFF\xFF\xFF\xF8')
        self.assertTrue(all(c.kwargs['num_msgs'] == 1
                            for c in iface.PassThruReadMsgs.call_args_list))
        iface.PassThruStopMsgFilter.assert_called_once()

    def test_fallback_probes_functional_and_physical_but_returns_physical_addresses(self):
        iface = Mock()
        iface.PassThruStartMsgFilter.side_effect = J2534Error(J2534Errors.ERR_INVALID_FILTER_ID)
        responses = [b'\x7F\x01\x11', b'\x41\x00\x00\x00\x00\x01',
                     b'\x41\x00\x00\x00\x00\x01'] + [None] * 12
        output = io.StringIO()
        with (patch.object(obd, 'setup_flow_filter', return_value=1) as setup,
              patch.object(obd, 'transact', side_effect=responses) as transact,
              contextlib.redirect_stdout(output)):
            self.assertEqual(obd.discover_ecus(iface, 1),
                             [(0x7E0, 0x7E8), (0x7E1, 0x7E9)])
        self.assertEqual(transact.call_count, 15)
        self.assertEqual([c.args[2:4] for c in transact.call_args_list[:3]],
                         [(0x7E0, 0x7E8), (0x7DF, 0x7E8), (0x7E1, 0x7E9)])
        setup.assert_any_call(iface, 1, 0x7E0, 0x7E8)
        self.assertIn("negative or incomplete response 7F 01 11", output.getvalue())
        self.assertIn("No valid Mode 01 PID 00 response from RX=0x7EA", output.getvalue())
        self.assertNotIn("No ECU responded to functional broadcast", output.getvalue())
        iface.PassThruWriteMsgs.assert_not_called()

    def test_discovery_surfaces_read_errors_and_stops_filter(self):
        iface = Mock()
        iface.PassThruReadMsgs.side_effect = J2534Error(J2534Errors.ERR_FAILED)
        with (patch.object(obd.time, 'time', return_value=0),
              contextlib.redirect_stdout(io.StringIO())):
            with self.assertRaises(J2534Error):
                obd.discover_ecus(iface, 1)
        iface.PassThruStopMsgFilter.assert_called_once()

    def test_functional_fallback_with_per_ecu_flow_filter(self):
        iface = Mock()
        state = {'rxid': None, 'request': None, 'time': 0}
        writes = []

        def start_filter(channel_id, filter_type, mask_msg, pattern_msg, flow_msg=None):
            if filter_type == obd.FilterType.PASS_FILTER:
                raise J2534Error(J2534Errors.ERR_INVALID_FILTER_ID)
            state['rxid'] = int.from_bytes(pattern_msg.Data, 'big')
            self.assertEqual(int.from_bytes(flow_msg.Data, 'big'), state['rxid'] - 8)
            return state['rxid']

        def write(channel_id, msgs, timeout):
            state['request'] = msgs[0].Data
            writes.append(state['request'])

        def read(channel_id, num_msgs, timeout):
            if state['rxid'] == 0x7E8 and state['request'] == b'\x00\x00\x07\xDF\x01\x00':
                return [SimpleNamespace(Data=struct.pack('>I', 0x7E8) +
                                        b'\x41\x00\x00\x00\x00\x01',
                                        ExtraData=b'', _RxStatus=0)]
            return []

        def clock():
            state['time'] += 0.6
            return state['time']

        iface.PassThruStartMsgFilter.side_effect = start_filter
        iface.PassThruWriteMsgs.side_effect = write
        iface.PassThruReadMsgs.side_effect = read
        with (patch.object(obd.time, 'time', side_effect=clock),
              contextlib.redirect_stdout(io.StringIO())):
            self.assertEqual(obd.discover_ecus(iface, 1), [(0x7E0, 0x7E8)])
        self.assertIn(b'\x00\x00\x07\xE0\x01\x00', writes)
        self.assertIn(b'\x00\x00\x07\xDF\x01\x00', writes)
        self.assertEqual(iface.PassThruStopMsgFilter.call_count, 8)


class ClearObdTests(unittest.TestCase):
    def setUp(self):
        self.iface = Mock()
        self.ecus = [(0x7E0, 0x7E8), (0x7E1, 0x7E9)]
        for name, value in (
            ('get_interfaces', {'adapter': 'dll'}),
            ('load_interface', self.iface),
            ('discover_ecus', self.ecus),
        ):
            mock = patch.object(clear, name, return_value=value)
            mock.start()
            self.addCleanup(mock.stop)

    def test_declined_confirmation_never_sends_clear(self):
        with (patch('builtins.input', return_value='no'),
              patch.object(clear, 'transact') as transact,
              contextlib.redirect_stdout(io.StringIO())):
            self.assertEqual(clear.main(), 0)
        transact.assert_not_called()
        self.iface.PassThruDisconnect.assert_called_once()
        self.iface.PassThruClose.assert_called_once()

    def test_confirmed_clear_targets_each_ecu_and_reports_partial_failure(self):
        output = io.StringIO()
        with (patch('builtins.input', return_value='yes'),
              patch.object(clear, 'setup_flow_filter', side_effect=[1, 2]),
              patch.object(clear, 'transact', side_effect=[b'\x44', None]) as transact,
              contextlib.redirect_stdout(output)):
            self.assertEqual(clear.main(), 1)
        channel = self.iface.PassThruConnect.return_value
        self.assertEqual(transact.call_args_list, [
            call(self.iface, channel, 0x7E0, 0x7E8, b'\x04',
                 timeout_ms=3000, expected_prefix=b'\x44'),
            call(self.iface, channel, 0x7E1, 0x7E9, b'\x04',
                 timeout_ms=3000, expected_prefix=b'\x44'),
        ])
        self.assertEqual(self.iface.PassThruStopMsgFilter.call_count, 2)
        self.assertIn("ECU accepted Mode 04", output.getvalue())
        self.assertIn("clearing outcome is unknown", output.getvalue())
        self.assertNotIn("All discovered ECUs accepted", output.getvalue())
        self.iface.PassThruDisconnect.assert_called_once()
        self.iface.PassThruClose.assert_called_once()

    def test_rejection_and_filter_error_are_reported(self):
        output = io.StringIO()
        with (patch('builtins.input', return_value='yes'),
              patch.object(clear, 'setup_flow_filter',
                           side_effect=[J2534Error(J2534Errors.ERR_FAILED), 2]),
              patch.object(clear, 'transact', return_value=b'\x7F\x04\x22') as transact,
              contextlib.redirect_stdout(output)):
            self.assertEqual(clear.main(), 1)
        self.assertEqual(transact.call_count, 1)
        self.assertIn("Could not set flow-control filter", output.getvalue())
        self.assertIn("Mode 04 rejected: 7F 04 22", output.getvalue())

    def test_success_and_no_discovery(self):
        with (patch('builtins.input', return_value='yes'),
              patch.object(clear, 'setup_flow_filter', return_value=1),
              patch.object(clear, 'transact', return_value=b'\x44'),
              contextlib.redirect_stdout(io.StringIO())):
            self.assertEqual(clear.main(), 0)
        with (patch.object(clear, 'discover_ecus', return_value=[]),
              patch('builtins.input') as prompt,
              patch.object(clear, 'transact') as transact,
              contextlib.redirect_stdout(io.StringIO())):
            self.assertEqual(clear.main(), 1)
        prompt.assert_not_called()
        transact.assert_not_called()

    def test_connect_failure_closes_device(self):
        self.iface.PassThruConnect.side_effect = J2534Error(J2534Errors.ERR_FAILED)
        with contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(J2534Error):
                clear.main()
        self.iface.PassThruDisconnect.assert_not_called()
        self.iface.PassThruClose.assert_called_once()


if __name__ == '__main__':
    unittest.main()
