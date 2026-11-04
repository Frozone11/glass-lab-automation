#!/usr/bin/env python3
"""Direct ROS1 navigation client for the *physical* MiR100.

Read-only / dry-run unless navigate_to(..., execute=True) is explicitly called.
Do not run Gazebo, a second move_base, or a local ROS master against the robot.
"""
from ros_config import configure_ros

configure_ros()

import math
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from mir_actions.msg import MirMoveBaseAction, MirMoveBaseGoal
from tf.transformations import quaternion_from_euler
from mission_validation import finite_number


class PhysicalMiRNavigation:
    def __init__(self, server_timeout=10.0):
        if not rospy.core.is_initialized():
            raise RuntimeError("Initialize rospy before constructing navigation")
        finite_number(server_timeout, "server_timeout", True)
        # Caller owns ROS initialization.
        self.client = actionlib.SimpleActionClient('/move_base', MirMoveBaseAction)
        if not self.client.wait_for_server(rospy.Duration(server_timeout)):
            raise RuntimeError('Physical MiR action server /move_base unavailable')

    @staticmethod
    def build_goal(x, y, degrees):
        for name, value in (("x", x), ("y", y), ("orientation", degrees)):
            finite_number(value, name)
        if not -180.0 <= degrees <= 180.0:
            raise ValueError('Orientation must be between -180 and 180 degrees')
        goal = MirMoveBaseGoal()
        goal.move_task = MirMoveBaseGoal.GLOBAL_MOVE
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)
        qx, qy, qz, qw = quaternion_from_euler(0, 0, math.radians(degrees))
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw
        # Leave MiR-specific tuning and collision fields at their defaults until
        # the robot's supported behavior is confirmed by the operator/vendor.
        # Todo: Verify MiR-specific collision settings before live navigation.
        # Do not enable live execution until goal semantics are validated.  
        return goal

    def navigate_to(self, x, y, orientation, execute=False, timeout=120.0):
        finite_number(timeout, "navigation timeout", True)
        goal = self.build_goal(x, y, orientation)
        if not execute:
            rospy.loginfo('Navigation: DRY RUN (goal not sent)')
            return True

        # Only call execute=True after map, localization, physical safety,
        # MiR mode and its goal semantics have been independently verified.
        goal.target_pose.header.stamp = rospy.Time.now()
        terminal = {GoalStatus.PREEMPTED, GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                    GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.LOST}
        names = {getattr(GoalStatus, name): name for name in (
            'PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED',
            'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST')}
        outstanding = False
        try:
            self.client.send_goal(goal)
            outstanding = True
            deadline = time.monotonic() + timeout
            previous = None
            while True:
                if rospy.is_shutdown():
                    raise RuntimeError('ROS shutdown during navigation')
                state = self.client.get_state()
                if state != previous:
                    rospy.loginfo('Navigation status: %s (%s)', names.get(state, state),
                                  self.client.get_goal_status_text())
                    previous = state
                if state in terminal:
                    # LOST is not evidence that the physical robot stopped.
                    outstanding = state == GoalStatus.LOST
                    return state == GoalStatus.SUCCEEDED
                if time.monotonic() >= deadline:
                    raise TimeoutError('Navigation timed out; physical stop is not confirmed')
                time.sleep(0.1)
        finally:
            if outstanding:
                rospy.logwarn('Requesting cancellation of this client goal only; stop is not confirmed')
                self.client.cancel_goal()
