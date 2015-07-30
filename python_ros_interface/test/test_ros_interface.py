#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import rostest
import rospy
from rospy_tutorials.srv import AddTwoIntsRequest
from rospy import ROSException
from ros_interface import ROSService

class TestService(unittest.TestCase):
    def test_success(self):
        add_two_ints = ROSService('/add_two_ints')
        self.assertEqual(add_two_ints(1, 2).sum, 3)
    def test_not_resolvable(self):
        with self.assertRaises(ROSException):
            add_two_ints = ROSService('/add_two_ints_2')
            add_two_ints(3, 4)
    def test_with_timeout(self):
        with self.assertRaises(ROSException):
            add_two_ints = ROSService('/add_two_ints_2', timeout=1.0)
            add_two_ints(3, 4)
    def test_get_request(self):
        add_two_ints = ROSService('/add_two_ints')
        self.assertIs(add_two_ints.request, AddTwoIntsRequest)

if __name__ == '__main__':
    import rostest
    rospy.init_node('test_ros_interface')
    rostest.rosrun('python_ros_interface', 'test_service', TestService)
