#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=no-member

import unittest
from mock import patch
import rostest
import logging
import rospy
import geometry_msgs
import tf.transformations
from ros_interface import rosmsg

class TestRosMsg(unittest.TestCase):
    @patch('rospy.Time.now')
    def test_TransformStamped(self, mock):
        mock.return_value = rospy.Time(1)
        t = rosmsg.Transform(x=1, y=2, z=3,
                             ai=0.1, aj=0.2, ak=0.3, axes='rxyz')
        q = tf.transformations.quaternion_from_euler(0.1, 0.2, 0.3, axes='rxyz')
        ts = rosmsg.TransformStamped(t, parent_id='parent', child_id='child')
        self.assertEqual(ts.transform.translation.x, 1)
        self.assertEqual(ts.transform.translation.y, 2)
        self.assertEqual(ts.transform.translation.z, 3)
        self.assertEqual(ts.transform.rotation.x, q[0])
        self.assertEqual(ts.transform.rotation.y, q[1])
        self.assertEqual(ts.transform.rotation.z, q[2])
        self.assertEqual(ts.transform.rotation.w, q[3])
        self.assertEqual(ts.header.frame_id, 'parent')
        self.assertEqual(ts.child_frame_id, 'child')
        self.assertEqual(ts.header.stamp, rospy.Time(1))

    @patch('rospy.Time.now')
    def test_Stamped(self, mock):
        mock.return_value = rospy.Time(1)
        s = rosmsg.Stamped(rosmsg.Vector3(x=1, y=2, z=3), frame_id='x')
        self.assertIsInstance(s, geometry_msgs.msg.Vector3Stamped)
        self.assertEqual(s.vector.x, 1)
        self.assertEqual(s.vector.y, 2)
        self.assertEqual(s.vector.z, 3)
        self.assertEqual(s.header.frame_id, 'x')
        self.assertEqual(s.header.stamp, rospy.Time(1))

    def test_JointTrajectoryPoint(self):
        jtp = rosmsg.JointTrajectoryPoint([0.1, 0.2], 1.0)
        self.assertEqual(jtp.positions, [0.1, 0.2])
        self.assertEqual(jtp.velocities, [0.0, 0.0])
        self.assertEqual(jtp.accelerations, [0.0, 0.0])
        self.assertEqual(jtp.time_from_start, rospy.Duration(1.0))

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

        p = rosmsg.Pose(position=(1, 2, 3),
                        orientation=(0, 0, 0, 1))
        self.assertEqual(p.position.x, 1)
        self.assertEqual(p.position.y, 2)
        self.assertEqual(p.position.z, 3)
        self.assertEqual(p.orientation.x, 0)
        self.assertEqual(p.orientation.y, 0)
        self.assertEqual(p.orientation.z, 0)
        self.assertEqual(p.orientation.w, 1)

        t = geometry_msgs.msg.Transform(geometry_msgs.msg.Vector3(1, 2, 3),
                                        geometry_msgs.msg.Quaternion(0, 1, 0, 0))
        p = rosmsg.Pose(transform=t)
        self.assertEqual(p.position.x, 1)
        self.assertEqual(p.position.y, 2)
        self.assertEqual(p.position.z, 3)
        self.assertEqual(p.orientation.x, 0)
        self.assertEqual(p.orientation.y, 1)
        self.assertEqual(p.orientation.z, 0)
        self.assertEqual(p.orientation.w, 0)

    def test_Transform(self):
        t = rosmsg.Transform(x=1, y=2, z=3,
                             ai=0.1, aj=0.2, ak=0.3, axes='rxyz')
        q = tf.transformations.quaternion_from_euler(0.1, 0.2, 0.3, axes='rxyz')
        self.assertEqual(t.translation.x, 1)
        self.assertEqual(t.translation.y, 2)
        self.assertEqual(t.translation.z, 3)
        self.assertEqual(t.rotation.x, q[0])
        self.assertEqual(t.rotation.y, q[1])
        self.assertEqual(t.rotation.z, q[2])
        self.assertEqual(t.rotation.w, q[3])

        t = rosmsg.Transform(translation=(1, 2, 3),
                             rotation=(0, 0, 0, 1))
        self.assertEqual(t.translation.x, 1)
        self.assertEqual(t.translation.y, 2)
        self.assertEqual(t.translation.z, 3)
        self.assertEqual(t.rotation.x, 0)
        self.assertEqual(t.rotation.y, 0)
        self.assertEqual(t.rotation.z, 0)
        self.assertEqual(t.rotation.w, 1)

        t1 = rosmsg.Transform(translation=(1, 2, 3),
                              rotation=(0, 0, 1, 0))
        t2 = rosmsg.Transform(translation=(2, 4, 6),
                              rotation=(0, 1, 0, 0))
        t = rosmsg.Transform(world=t1, local=t2)
        self.assertAlmostEqual(t.translation.x, -1)
        self.assertAlmostEqual(t.translation.y, -2)
        self.assertAlmostEqual(t.translation.z, 9)
        self.assertAlmostEqual(t.rotation.x, 1)
        self.assertAlmostEqual(t.rotation.y, 0)
        self.assertAlmostEqual(t.rotation.z, 0)
        self.assertAlmostEqual(t.rotation.w, 0)

        t1 = rosmsg.Transform(translation=(1, 2, 3),
                              rotation=(0, 0, 1, 0))
        t = rosmsg.Transform(inverse=t1)
        t2 = rosmsg.Transform(world=t1, local=t)
