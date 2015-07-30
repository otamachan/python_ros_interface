#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import rostest
import rospy
from ros_interface import ROSService

class TestService(unittest.TestCase):
    def test_dummy(self):
        add_two_ints = ROSService('/add_two_ints')
        self.assertEqual(add_two_ints(1, 2).sum, 3)

if __name__ == '__main__':
    import rostest
    rospy.init_node('test_ros_interface')
    rostest.rosrun('python_ros_interface', 'test_service', TestService)
