#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=no-member

import unittest
import rostest
import logging
import rospy
import geometry_msgs
import tf.transformations
from ros_interface import rosmsg

class TestRosMsg(unittest.TestCase):
    def test_Vector3(self):
        v = rosmsg.Vector3(x=1, y=2, z=3)
        self.assertEqual(v.x, 1)
        self.assertEqual(v.y, 2)
        self.assertEqual(v.z, 3)

        v = rosmsg.Vector3(vector3=(1, 2, 3))
        self.assertEqual(v.x, 1)
        self.assertEqual(v.y, 2)
        self.assertEqual(v.z, 3)

        v = rosmsg.Vector3(vector3=geometry_msgs.msg.Vector3(1, 2, 3))
        self.assertEqual(v.x, 1)
        self.assertEqual(v.y, 2)
        self.assertEqual(v.z, 3)

        with self.assertRaises(ValueError):
            v = rosmsg.Vector3(vector3=(1, 2, 3, 4))

    def test_Point(self):
        v = rosmsg.Point(x=1, y=2, z=3)
        self.assertEqual(v.x, 1)
        self.assertEqual(v.y, 2)
        self.assertEqual(v.z, 3)

        v = rosmsg.Point(point=(1, 2, 3))
        self.assertEqual(v.x, 1)
        self.assertEqual(v.y, 2)
        self.assertEqual(v.z, 3)

        v = rosmsg.Point(point=geometry_msgs.msg.Point(1, 2, 3))
        self.assertEqual(v.x, 1)
        self.assertEqual(v.y, 2)
        self.assertEqual(v.z, 3)

        with self.assertRaises(ValueError):
            v = rosmsg.Point(point=(1, 2, 3, 4))

    def test_Quaternion(self):
        q = rosmsg.Quaternion(ai=0.1, aj=0.2, ak=0.3, axes='rxyz')
        q2 = tf.transformations.quaternion_from_euler(0.1, 0.2, 0.3, axes='rxyz')
        self.assertEqual(q.x, q2[0])
        self.assertEqual(q.y, q2[1])
        self.assertEqual(q.z, q2[2])
        self.assertEqual(q.w, q2[3])

        q = rosmsg.Quaternion(quaternion=(0, 1, 0, 0))
        self.assertEqual(q.x, 0)
        self.assertEqual(q.y, 1)
        self.assertEqual(q.z, 0)
        self.assertEqual(q.w, 0)

        q = rosmsg.Quaternion(quaternion=geometry_msgs.msg.Quaternion(0, 0, 1, 0))
        self.assertEqual(q.x, 0)
        self.assertEqual(q.y, 0)
        self.assertEqual(q.z, 1)
        self.assertEqual(q.w, 0)

        with self.assertRaises(ValueError):
            q = rosmsg.Quaternion(quaternion=(2, 3, 4))

    def test_Pose(self):
        p = rosmsg.Pose(x=1, y=2, z=3,
                        ai=0.1, aj=0.2, ak=0.3, axes='rxyz')
        q = tf.transformations.quaternion_from_euler(0.1, 0.2, 0.3, axes='rxyz')
        self.assertEqual(p.position.x, 1)
        self.assertEqual(p.position.y, 2)
        self.assertEqual(p.position.z, 3)
        self.assertEqual(p.orientation.x, q[0])
        self.assertEqual(p.orientation.y, q[1])
        self.assertEqual(p.orientation.z, q[2])
        self.assertEqual(p.orientation.w, q[3])

        p = rosmsg.Pose(x=1, y=2, z=3,
                        ai=0.1, aj=0.2, ak=0.3, axes='rxyz')
