#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from rospy_tutorials.srv import AddTwoInts, AddTwoIntsResponse
from actionlib_tutorials.msg import FibonacciAction, FibonacciFeedback, FibonacciResult
from std_msgs.msg import Int32
class Mock(object):
    def __init__(self):
        self._service_server = rospy.Service('add_two_ints', AddTwoInts, self.add_two_ints)
        self._feedback = FibonacciFeedback()
        self._result = FibonacciResult()
        self._publisher = rospy.Publisher('counter', Int32, queue_size=1)
        self._actionlib_server = actionlib.SimpleActionServer('fibonacci', FibonacciAction, execute_cb=self.execute_cb, auto_start=False)
        self._actionlib_server.start()

    def add_two_ints(self, req):
        return AddTwoIntsResponse(req.a + req.b)

    def execute_cb(self, goal):
        r = rospy.Rate(4)
        success = True

        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)

        for i in xrange(1, goal.order):
            if self._actionlib_server.is_preempt_requested():
                self._actionlib_server.set_preempted()
                success = False
                break
            self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            self._actionlib_server.publish_feedback(self._feedback)
            r.sleep()

        if success:
            self._result.sequence = self._feedback.sequence
            self._actionlib_server.set_succeeded(self._result)

if __name__ == "__main__":
    rospy.init_node('mock')
    m = Mock()
    rospy.spin()
