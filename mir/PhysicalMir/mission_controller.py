#!/usr/bin/env python3
"""Mission orchestrator: ROS MiR navigation + optional ROS oven services.

Defaults to dry-run. No movement or oven service calls unless --execute
and --confirm-safe are BOTH supplied and the robot's map/state checks pass.
"""

from ros_config import configure_ros

configure_ros()

import argparse
import json
import os
import urllib.request
import time
from mission_validation import validate_config
from pathlib import Path



class OvenInterface:
    def __init__(self):
        self.status = None
        rospy.Subscriber('/oven/status', String, self._on_status)
        rospy.wait_for_service('/oven/open', timeout=8)
        rospy.wait_for_service('/oven/close', timeout=8)
        self.open = rospy.ServiceProxy('/oven/open', Trigger)
        self.close = rospy.ServiceProxy('/oven/close', Trigger)

    def _on_status(self, msg):
        self.status = msg.data.strip().upper()

    def wait_status(self, wanted, timeout=30):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.status == wanted:
                return
            time.sleep(0.2)
        raise TimeoutError('Oven did not report ' + wanted)

    def cycle(self):
        self.wait_status('CLOSED')
        res = self.open()
        if not res.success:
            raise RuntimeError('Oven open failed: ' + res.message)
        self.wait_status('OPEN')
        res = self.close()
        if not res.success:
            raise RuntimeError('Oven close failed: ' + res.message)
        self.wait_status('CLOSED')


def check_robot_before_motion(expected_map_id, *, execute=False, confirm_safe=False):
    """Only initial preflight may opt into resuming an operator-confirmed Pause."""
    url = 'http://192.168.12.20/api/v2.0.0/status'
    authorization = os.environ.get('MIR_API_AUTHORIZATION')
    get_request = urllib.request.Request(url, headers={'Authorization': authorization}) if authorization else url
    deadline = None
    while True:
        if deadline is not None and time.monotonic() >= deadline:
            raise TimeoutError('MiR Pause to Ready transition timed out')
        request_timeout = 5 if deadline is None else min(5, deadline - time.monotonic())
        if request_timeout <= 0:
            raise TimeoutError('MiR Pause to Ready transition timed out')
        with urllib.request.urlopen(get_request, timeout=request_timeout) as response:
            status = json.load(response)
        if deadline is not None and time.monotonic() >= deadline:
            raise TimeoutError('MiR Pause to Ready transition timed out')
        rospy.loginfo('Robot state=%s map=%s', status.get('state_text'), status.get('map_id'))
        if status.get('map_id') != expected_map_id:
            raise RuntimeError('Active MiR map does not match approved map_id')
        if status.get('mode_text') != 'Mission' or status.get('errors') != []:
            raise RuntimeError('MiR must be in Mission mode with no active errors')
        state = (status.get('state_id'), status.get('state_text'))
        if state == (3, 'Ready'):
            return
        if state != (4, 'Pause') or not (execute and confirm_safe):
            raise RuntimeError('MiR is not Ready in Mission mode')
        if deadline is None:
            # MiR100 2.13.5.4 API details supplied by the operator: PUT status,
            # state_id=3 requests Ready; never send clear_error or safety resets.
            if not authorization:
                raise RuntimeError('Missing MiR API authentication: refusing automatic resume')
            request = urllib.request.Request(
                url, data=b'{"state_id": 3}', method='PUT',
                headers={'Content-Type': 'application/json',
                         'Authorization': authorization})
            rospy.loginfo('Robot state transition: Pause -> Ready')
            deadline = time.monotonic() + 10
            with urllib.request.urlopen(request, timeout=5):
                pass
        time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=str(Path(__file__).with_name('mission_config.json')))
    parser.add_argument('--validate-only', action='store_true', help='Validate JSON locally without ROS or network access')
    parser.add_argument('--execute', action='store_true', help='Actually send commands to physical hardware')
    parser.add_argument('--confirm-safe', action='store_true', help='Operator confirms safe to resume, area, localization, E-stop, goals')
    args, remaining = parser.parse_known_args()
    if any(':=' not in arg for arg in remaining):
        parser.error('Unknown arguments: ' + ' '.join(remaining))
    if args.execute and args.validate_only:
        parser.error('--validate-only cannot be combined with --execute')
    config = validate_config(json.loads(Path(args.config).read_text()), live=args.execute)
    if args.validate_only:
        print('Configuration structure valid; map geometry, localization and goal semantics remain operator checks.')
        return
    global rospy, String, Trigger
    import rospy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from physical_mir_navigation import PhysicalMiRNavigation
    rospy.init_node('vt3_physical_mir_mission')

    waypoints = config['waypoints']
    if not waypoints:
        raise ValueError('Add inspected, map-valid waypoints first')
    if args.execute:
        if not args.confirm_safe:
            raise RuntimeError('Missing --confirm-safe: refusing hardware commands')
        map_id = config.get('approved_map_id')
        if not map_id or map_id == 'REPLACE_AFTER_MAP_READY':
            raise RuntimeError('Set approved_map_id before enabling movement')
        check_robot_before_motion(map_id, execute=args.execute, confirm_safe=args.confirm_safe)

    
    nav = PhysicalMiRNavigation()
    oven = None
    for i, wp in enumerate(waypoints, 1):
        x, y, orientation, use_oven = wp['x'], wp['y'], wp['orientation_deg'], wp.get('oven', False)
        rospy.loginfo('Waypoint %d/%d: x=%.3f y=%.3f deg=%.1f oven=%s', i, len(waypoints), x, y, orientation, use_oven)
        if args.execute:
            check_robot_before_motion(config['approved_map_id'])
        ok = nav.navigate_to(x, y, orientation, execute=args.execute,
                             timeout=float(config.get('navigation_timeout_seconds', 120)))
        if not ok:
            raise RuntimeError('Navigation did not succeed at waypoint ' + str(i))
        rospy.loginfo('Waypoint %d/%d %s', i, len(waypoints),
                      'navigation succeeded' if args.execute else 'dry-run validated (not navigated)')
        if use_oven:
            if args.execute:
                if oven is None:
                    oven = OvenInterface()
                rospy.loginfo('Waypoint %d: starting oven cycle', i)
                oven.cycle()
                rospy.loginfo('Waypoint %d: oven cycle completed', i)
            else:
                rospy.loginfo('DRY RUN: oven cycle NOT SENT')
    rospy.loginfo('Mission finished (%s)', 'LIVE' if args.execute else 'DRY RUN')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('Mission failed: ' + str(exc))
        raise SystemExit(1)
