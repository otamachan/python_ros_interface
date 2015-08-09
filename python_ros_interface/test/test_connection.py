#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=no-member

import os
import unittest
import rospy
import subprocess
from ros_interface import SubscribeManager
import utils

class TestSubscribeManager(unittest.TestCase):
    #
    @classmethod
    def setUpClass(cls):
        print '+'*100
        test_path = os.path.dirname(__file__)
        cls.proc = utils.launch(os.path.join(test_path, 'ros_interface.test'))

    @classmethod
    def tearDownClass(cls):
        utils.wait_shutdown(cls.proc)
        print '-'*100

    def test_singleton(self):
        sub1 = SubscribeManager()
        sub2 = SubscribeManager()
        self.assertEqual(sub1.tf, sub2.tf)
