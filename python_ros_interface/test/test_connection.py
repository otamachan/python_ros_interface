#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=no-member

import os
import unittest
import rospy
import subprocess
from ros_interface import SubscribeManager

class TestSubscribeManager(unittest.TestCase):
    #
    @classmethod
    def setUpClass(cls):
        print '+'*100
        test_path = os.path.dirname(__file__)
        cls.proc = subprocess.Popen(['roslaunch', 'ros_interface.test'], cwd=test_path)
        rospy.init_node('test_connection')

    @classmethod
    def tearDownClass(cls):
        cls.proc.terminate()
        cls.proc.wait()
        print '-'*100

    def test_singleton(self):
        sub1 = SubscribeManager()
        sub2 = SubscribeManager()
        self.assertEqual(sub1.tf, sub2.tf)
