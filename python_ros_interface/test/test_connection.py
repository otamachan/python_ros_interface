#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=no-member

import os
import unittest
from ros_interface import SubscribeManager
from ros_interface import rostest_launch

class TestSubscribeManager(unittest.TestCase):
    #
    @classmethod
    def setUpClass(cls):
        rostest_launch(os.path.join(
            os.path.dirname(__file__), 'ros_interface.test'))

    def test_singleton(self):
        sub1 = SubscribeManager()
        sub2 = SubscribeManager()
        self.assertEqual(sub1.tf, sub2.tf)
