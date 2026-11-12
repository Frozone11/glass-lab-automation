import copy
import importlib
import json
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from mission_validation import validate_config


class ValidationTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads(Path('mission_config.json').read_text())

    def test_config_valid_without_verification_flags(self):
        validate_config(self.config)
        validate_config(self.config, live=True)

    def test_placeholder_map_blocks_live_execution(self):
        self.config["approved_map_id"] = "REPLACE_AFTER_MAP_READY"
        validate_config(self.config)
        with self.assertRaises(ValueError):
            validate_config(self.config, live=True)

    def test_invalid_values(self):
        for key, value in [('x', True), ('y', float('nan')), ('x', '1'),
                           ('orientation_deg', 181), ('oven', 'false')]:
            config = copy.deepcopy(self.config)
            config['waypoints'][0][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                validate_config(config)
        for value in (0, -1, float('inf'), True):
            self.config['navigation_timeout_seconds'] = value
            with self.assertRaises(ValueError):
                validate_config(self.config)


class MissionTests(unittest.TestCase):
    def test_failed_navigation_blocks_oven_and_next_waypoint(self):
        import mission_controller as mission
        config = json.loads(Path('mission_config.json').read_text())
        config.update(approved_map_id='12345678-1234-1234-1234-123456789abc')
        config['waypoints'] = [dict(x=1, y=2, orientation_deg=0, oven=True)] * 2
        rospy = MagicMock()
        navigation = MagicMock()
        navigation.PhysicalMiRNavigation.return_value.navigate_to.return_value = False
        modules = {'rospy': rospy, 'std_msgs': MagicMock(), 'std_msgs.msg': MagicMock(),
                   'std_srvs': MagicMock(), 'std_srvs.srv': MagicMock(),
                   'physical_mir_navigation': navigation}
        with patch.dict(sys.modules, modules), patch.object(Path, 'read_text', return_value=json.dumps(config)), \
             patch.object(sys, 'argv', ['mission_controller.py', '--execute', '--confirm-safe']), \
             patch.object(mission, 'check_robot_before_motion'), patch.object(mission, 'OvenInterface') as oven:
            with self.assertRaises(RuntimeError):
                mission.main()
            rospy.init_node.assert_called_once()
            navigation.PhysicalMiRNavigation.return_value.navigate_to.assert_called_once()
            oven.assert_not_called()


    def test_execution_gate_and_later_pause(self):
        import mission_controller as mission
        config = json.loads(Path('mission_config.json').read_text())
        config['waypoints'] *= 2
        for flags in ([], ['--confirm-safe'], ['--execute'], ['--execute', '--confirm-safe']):
            rospy = MagicMock()
            navigation = MagicMock()
            modules = {'rospy': rospy, 'std_msgs.msg': MagicMock(),
                       'std_srvs.srv': MagicMock(), 'physical_mir_navigation': navigation}
            with self.subTest(flags=flags), patch.dict(sys.modules, modules), \
                 patch.object(Path, 'read_text', return_value=json.dumps(config)), \
                 patch.object(sys, 'argv', ['mission_controller.py'] + flags), \
                 patch.object(mission, 'check_robot_before_motion') as check, \
                 patch.object(mission.urllib.request, 'urlopen') as network:
                nav = navigation.PhysicalMiRNavigation.return_value
                if flags == ['--execute']:
                    with self.assertRaisesRegex(RuntimeError, 'confirm-safe'):
                        mission.main()
                    check.assert_not_called()
                    nav.navigate_to.assert_not_called()
                elif '--execute' in flags:
                    # Initial preflight and first waypoint pass; Pause at the
                    # second waypoint aborts before another goal is sent.
                    check.side_effect = [None, None, RuntimeError('MiR is not Ready')]
                    with self.assertRaisesRegex(RuntimeError, 'not Ready'):
                        mission.main()
                    nav.navigate_to.assert_called_once()
                    self.assertEqual(check.call_args_list[0].kwargs,
                                     {'execute': True, 'confirm_safe': True})
                    self.assertTrue(all(not call.kwargs for call in check.call_args_list[1:]))
                else:
                    mission.main()
                    check.assert_not_called()
                    self.assertEqual(nav.navigate_to.call_count, 2)
                    for call in nav.navigate_to.call_args_list:
                        self.assertFalse(call.kwargs['execute'])
                network.assert_not_called()


class PreflightTests(unittest.TestCase):
    def setUp(self):
        import mission_controller
        self.mission = mission_controller
        environment = patch.dict(self.mission.os.environ, {}, clear=True)
        environment.start()
        self.addCleanup(environment.stop)
        self.status = dict(state_id=3, state_text='Ready', mode_text='Mission',
                           map_id='approved', errors=[])
        self.response = MagicMock()
        self.response.__enter__.return_value = self.response
        self.network = patch.object(self.mission.urllib.request, 'urlopen', return_value=self.response)
        self.urlopen = self.network.start()
        self.addCleanup(self.network.stop)
        self.logging = patch.object(self.mission, 'rospy', MagicMock(), create=True)
        self.logging.start()
        self.addCleanup(self.logging.stop)

    def check(self):
        with patch.object(self.mission.json, 'load', return_value=self.status):
            self.mission.check_robot_before_motion('approved')

    def test_ready_and_matching_map(self):
        self.check()
        self.urlopen.assert_called_once_with(
            'http://192.168.12.20/api/v2.0.0/status', timeout=5)

    def test_unsupported_states_never_write(self):
        for state in ('Pause', 'EmergencyStop', 'Error', 'Manualcontrol', 'Executing', None):
            self.status['state_text'] = state
            with self.subTest(state=state), self.assertRaises(RuntimeError):
                self.check()
        # Existing preflight only sends GET requests, including on rejection.
        self.assertTrue(all(isinstance(call.args[0], str) for call in self.urlopen.call_args_list))

    def resume(self, statuses):
        with patch.object(self.mission.json, 'load', side_effect=statuses), \
             patch.dict(self.mission.os.environ, {'MIR_API_AUTHORIZATION': 'Basic mocked'}), \
             patch.object(self.mission.time, 'sleep'):
            self.mission.check_robot_before_motion('approved', execute=True, confirm_safe=True)

    def pause(self, **changes):
        return dict(self.status, state_id=4, state_text='Pause', **changes)

    def writes(self):
        return [call.args[0] for call in self.urlopen.call_args_list
                if getattr(call.args[0], 'method', None) == 'PUT']

    def test_pause_to_ready(self):
        self.resume([self.pause(), self.pause(), self.status])
        request, = self.writes()
        self.assertEqual(request.full_url, 'http://192.168.12.20/api/v2.0.0/status')
        self.assertEqual(json.loads(request.data), {'state_id': 3})
        self.assertEqual(request.get_header('Authorization'), 'Basic mocked')
        self.assertEqual(request.get_header('Content-type'), 'application/json')
        self.assertEqual(self.urlopen.call_count, 4)

    def test_ready_does_not_resume(self):
        self.resume([self.status])
        self.assertEqual(self.writes(), [])

    def test_pause_requires_both_flags(self):
        for execute, confirm in ((False, False), (False, True), (True, False)):
            with self.subTest(execute=execute, confirm=confirm), \
                 patch.object(self.mission.json, 'load', return_value=self.pause()), \
                 self.assertRaises(RuntimeError):
                self.mission.check_robot_before_motion('approved', execute=execute, confirm_safe=confirm)
        self.assertEqual(self.writes(), [])

    def test_missing_authentication_blocks_resume(self):
        with patch.object(self.mission.json, 'load', return_value=self.pause()), \
             self.assertRaisesRegex(RuntimeError, 'authentication'):
            self.mission.check_robot_before_motion('approved', execute=True, confirm_safe=True)
        self.assertEqual(self.writes(), [])

    def test_resume_timeout(self):
        with patch.object(self.mission.time, 'monotonic', side_effect=[0, 11]), \
             self.assertRaises(TimeoutError):
            self.resume([self.pause()])
        self.assertEqual(len(self.writes()), 1)

    def test_resume_api_failure(self):
        from urllib.error import URLError
        self.urlopen.side_effect = [self.response, URLError('PUT failed')]
        with self.assertRaises(URLError):
            self.resume([self.pause()])
        self.assertEqual(len(self.writes()), 1)

    def test_poll_api_failure(self):
        from urllib.error import URLError
        self.urlopen.side_effect = [self.response, self.response, URLError('GET failed')]
        with self.assertRaises(URLError):
            self.resume([self.pause()])
        self.assertEqual(len(self.writes()), 1)

    def test_ready_after_deadline_is_rejected(self):
        with patch.object(self.mission.time, 'monotonic', side_effect=[0, 1, 1, 11]), \
             self.assertRaises(TimeoutError):
            self.resume([self.pause(), self.status])
        self.assertEqual(len(self.writes()), 1)

    def test_resume_rejects_unsafe_initial_and_polled_status(self):
        bad_statuses = [dict(self.status, state_id=10, state_text='EmergencyStop'),
                        dict(self.status, state_id=12, state_text='Error'),
                        dict(self.status, state_id=5, state_text='Executing'),
                        dict(self.status, state_id=4),
                        self.pause(errors=[{'code': 1}]),
                        self.pause(errors=None),
                        self.pause(map_id='other'), self.pause(mode_text='Manual')]
        for bad in bad_statuses:
            for after_put in (False, True):
                self.urlopen.reset_mock()
                with self.subTest(status=bad, after_put=after_put), self.assertRaises(RuntimeError):
                    self.resume([self.pause(), bad] if after_put else [bad])
                self.assertEqual(len(self.writes()), int(after_put))

    def test_map_or_mode_mismatch(self):
        for field, value in (('map_id', 'other'), ('mode_text', 'Manual')):
            with self.subTest(field=field):
                old = self.status[field]
                self.status[field] = value
                with self.assertRaises(RuntimeError):
                    self.check()
                self.status[field] = old

    def test_api_errors_and_timeout_abort(self):
        from urllib.error import HTTPError, URLError
        for error in (HTTPError('status', 401, 'Unauthorized', {}, None),
                      URLError('unreachable'), TimeoutError('REST timeout')):
            self.urlopen.side_effect = error
            with self.subTest(error=error), self.assertRaises(type(error)):
                self.check()
            if isinstance(error, HTTPError):
                error.close()


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.status = types.SimpleNamespace(**{name: i for i, name in enumerate([
            'PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED', 'REJECTED',
            'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST'])})
        self.ros = MagicMock()
        self.ros.is_shutdown.return_value = False
        modules = {'rospy': self.ros, 'actionlib': MagicMock(),
                   'actionlib_msgs': MagicMock(),
                   'actionlib_msgs.msg': types.SimpleNamespace(GoalStatus=self.status),
                   'mir_actions': MagicMock(), 'mir_actions.msg': MagicMock(),
                   'tf': MagicMock(), 'tf.transformations': MagicMock()}
        with patch.dict(sys.modules, modules):
            sys.modules.pop('physical_mir_navigation', None)
            self.module = importlib.import_module('physical_mir_navigation')
        self.nav = self.module.PhysicalMiRNavigation.__new__(self.module.PhysicalMiRNavigation)
        self.nav.client = MagicMock()
        self.nav.build_goal = MagicMock()

    def tearDown(self):
        sys.modules.pop('physical_mir_navigation', None)

    def test_dry_run_has_no_commands(self):
        self.assertTrue(self.nav.navigate_to(0, 0, 0))
        self.nav.client.send_goal.assert_not_called()
        self.nav.client.cancel_goal.assert_not_called()

    def test_terminal_states(self):
        for state in ('SUCCEEDED', 'ABORTED', 'REJECTED', 'PREEMPTED', 'RECALLED', 'LOST'):
            self.nav.client.reset_mock()
            self.nav.client.get_state.return_value = getattr(self.status, state)
            self.assertEqual(self.nav.navigate_to(0, 0, 0, execute=True), state == 'SUCCEEDED')
            self.assertEqual(self.nav.client.cancel_goal.call_count, int(state == 'LOST'))
            self.nav.client.cancel_all_goals.assert_not_called()

    def test_timeout_and_interrupt_cancel_only_own_goal(self):
        self.nav.client.get_state.return_value = self.status.ACTIVE
        with patch.object(self.module.time, 'monotonic', side_effect=[0, 121]):
            with self.assertRaises(TimeoutError):
                self.nav.navigate_to(0, 0, 0, execute=True)
        self.nav.client.cancel_goal.assert_called_once()
        self.nav.client.reset_mock()
        self.nav.client.get_state.side_effect = KeyboardInterrupt
        with self.assertRaises(KeyboardInterrupt):
            self.nav.navigate_to(0, 0, 0, execute=True)
        self.nav.client.cancel_goal.assert_called_once()
        self.nav.client.cancel_all_goals.assert_not_called()


if __name__ == '__main__':
    unittest.main()
