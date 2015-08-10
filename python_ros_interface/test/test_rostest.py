#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import unittest
import mock
import rospy
from ros_interface.rostest import _Master, _wait_nodes

class TestROSTest(unittest.TestCase):
    def test_normal(self):
        master = _Master()
        launch_file = os.path.join(os.path.dirname(__file__), 'ros_interface.test')
        master.launch(launch_file, init_node=False)
        master.launch(launch_file, init_node=False)
        master = None

    def test_abnormal(self):
        master = _Master()
        launch_file = os.path.join(os.path.dirname(__file__), 'ros_interface.test')
        master.launch(launch_file)
        with self.assertRaises(RuntimeError):
            _wait_nodes('/dummy', timeout=0.0)
        with self.assertRaises(RuntimeError):
            with mock.patch('rospy.is_shutdown') as is_shutdown:
                is_shutdown.return_value = True
                _wait_nodes('/dummy')
