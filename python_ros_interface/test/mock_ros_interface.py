#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from rospy_tutorials.srv import AddTwoInts, AddTwoIntsResponse
from actionlib_tutorials.msg import FibonacciAction, FibonacciFeedback, FibonacciResult
from std_msgs.msg import Int32, String

class Mock(object):
    def __init__(self):
        self._service_server = rospy.Service('add_two_ints', AddTwoInts, self.add_two_ints)
        self._feedback = FibonacciFeedback()
        self._result = FibonacciResult()
        self._actionlib_server = actionlib.SimpleActionServer(
            'fibonacci',
            FibonacciAction, execute_cb=self.action_execute, auto_start=False)
        self._subscriber_dummy = rospy.Subscriber('fibonacci_dummy/goal', Int32)
        self._actionlib_server.start()
        self._publisher = rospy.Publisher('counter_pub', Int32, queue_size=1)
        self._publisher2 = rospy.Publisher('counter_pub2', Int32, queue_size=1)
        self._subscriber = rospy.Subscriber('counter_sub', Int32, callback=self.counter_callback)
        self._subscriber_control = rospy.Subscriber('mock_control', String, callback=self.mock_control_callback)
        self._publisher_echo = rospy.Publisher('counter_echo', Int32, queue_size=1, latch=True)
        self._timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback)
        self._counter = 0

    def add_two_ints(self, req):
        return AddTwoIntsResponse(req.a + req.b)

    def action_execute(self, goal):
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

    def action_execute_delayed(self, goal):
        feedback = FibonacciFeedback()
        result = FibonacciResult()
        feedback.sequence = []
        feedback.sequence.append(0)
        feedback.sequence.append(1)

        for i in xrange(1, goal.order):
            feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            self._actionlib_server_delayed.publish_feedback(feedback)

        result.sequence = feedback.sequence
        self._actionlib_server_delayed.set_succeeded(result)

    def timer_callback(self, event):
        self._publisher.publish(self._counter)
        self._counter += 1

    def counter_callback(self, data):
        self._publisher_echo.publish(data.data+1)

    def mock_control_callback(self, data):
        if data.data == "service":
            rospy.sleep(1.0)
            self._service_server_delayed = rospy.Service('add_two_ints_delayed', AddTwoInts, self.add_two_ints)
        elif data.data == "action":
            rospy.sleep(1.0)
            self._actionlib_server_delayed = actionlib.SimpleActionServer(
                'fibonacci_delayed',
                FibonacciAction, execute_cb=self.action_execute_delayed, auto_start=False)
            self._actionlib_server_delayed.start()
        elif data.data == "publish":
            rospy.sleep(1.0)
            self._publisher_delayed = rospy.Publisher('counter_pub_delayed', Int32, queue_size=1)
            while self._publisher_delayed.get_num_connections() == 0:
                rospy.sleep(0.2)
            self._publisher_delayed.publish(1)
            rospy.sleep(0.5)
            self._publisher_delayed.publish(2)
        elif data.data == "subscribe":
            rospy.sleep(1.0)
            self._subscriber_delayed = rospy.Subscriber('counter_sub_delayed', Int32, callback=self.counter_callback)

if __name__ == "__main__":
    rospy.init_node('mock')
    m = Mock()
    rospy.spin()
