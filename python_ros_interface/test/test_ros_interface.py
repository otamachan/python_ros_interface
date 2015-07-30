#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import rostest
import logging
import rospy
from rospy_tutorials.srv import AddTwoIntsRequest
from rospy import ROSException
from ros_interface import ROSService, ROSAction, ROSInterfaceRuntimeError

class TestService(unittest.TestCase):
    def test_success(self):
        add_two_ints = ROSService('/add_two_ints')
        self.assertEqual(add_two_ints(1, 2).sum, 3)

    def test_get_request(self):
        add_two_ints = ROSService('/add_two_ints')
        self.assertIs(add_two_ints.request, AddTwoIntsRequest)

    def test_fail_with_notresolvable(self):
        with self.assertRaises(ROSException):
            add_two_ints = ROSService('/add_two_ints_2')
            start = rospy.Time.now()
            add_two_ints(3, 4)
        self.assertGreater(rospy.Time.now() - start, rospy.Duration(1.0))

    def test_fail_with_timeout(self):
        with self.assertRaises(ROSException):
            add_two_ints = ROSService('/add_two_ints_2', timeout=0.5)
            start = rospy.Time.now()
            add_two_ints(3, 4)
        self.assertLess(rospy.Time.now() - start, rospy.Duration(1.0))

class TestAction(unittest.TestCase):
    def test_success(self):
        fibonacci = ROSAction('/fibonacci')
        self.assertEqual(fibonacci(5).sequence, (0, 1, 1, 2, 3, 5))

    def test_not_resolvable(self):
        fibonacci = ROSAction('/fibonacci_2')
        with self.assertRaises(ROSInterfaceRuntimeError):
            fibonacci(2)
            self.assertEqual(fibonacci.get_state(), fibonacci.SUCCEEDED)

    def test_fail_timouet(self):
        fibonacci = ROSAction('/fibonacci')
        fibonacci(100, timeout=1.0)
        self.assertEqual(fibonacci.get_state(), fibonacci.PREEMPTED)


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_ros_interface')
    logging.getLogger('rosout').setLevel(logging.DEBUG)
    #rostest.rosrun('python_ros_interface', 'test_service', TestService)
    rostest.rosrun('python_ros_interface', 'test_action', TestAction)
