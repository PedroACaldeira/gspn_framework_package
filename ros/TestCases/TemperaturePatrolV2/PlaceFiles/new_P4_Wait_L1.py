#! /usr/bin/env python3
import roslib
import rospy
import actionlib
import time
import psutil

import gspn_framework_package.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class MinimalActionServer(object):

    _feedback = gspn_framework_package.msg.ExecGSPNFeedback()
    _result = gspn_framework_package.msg.ExecGSPNResult()

    def __init__(self, name, _namespace):
        self._action_name = name
        self._namespace = _namespace
        self._as = actionlib.SimpleActionServer(self._action_name, gspn_framework_package.msg.ExecGSPNAction,
                                                execute_cb=self.execute_callback, auto_start = False)
        self._as.start()
        rospy.loginfo('WAIT L1 SERVER : ONLINE')
        self._as.start()

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accepts or rejects a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('WAIT L1 SERVER : Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accepts or rejects a client request to cancel an action."""
        self.get_logger().info('WAIT L1 SERVER : Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal):
        """Executes a goal."""
        rospy.loginfo('WAIT L1 SERVER : WAITING ON L1...')

        success = True

        self._feedback.time_passed = []

        # Start executing the action
        for i in range(1, 6):
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                success = False
                rospy.loginfo('WAIT L1 SERVER : Goal canceled')
                break
            self._feedback.time_passed.append(i)
            self._as.publish_feedback(self._feedback)
            time.sleep(1)

        if success:
            self._result.transition = 't5:Wait_Done_L1'
            self._as.set_succeeded(self._result)

        return self._result.transition


if __name__ == '__main__':
    rospy.init_node('action_server_4')
    _namespace = str(rospy.get_name()).split("/")[1]
    minimal_action_server = MinimalActionServer(rospy.get_name(), _namespace)
    rospy.spin()
