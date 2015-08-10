#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=no-member

import os
import unittest
import mock
from ros_interface import SubscribeManager, ROSTopic
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

    def test_init(self):
        with mock.patch('rospy.core.is_initialized') as is_initialized:
            with mock.patch('rospy.init_node') as init_node:
                is_initialized.return_value = False
                sub = SubscribeManager()
                init_node.assert_called_once_with('ros_interface', anonymous=True)

    def test_subscribe(self):
        sub = SubscribeManager()
        topic = ROSTopic('/counter_pub')
        sub.add_subscriber(topic)
        sub.subscribe()
        first = topic.get()
        second = topic.get()
        self.assertEqual(first.data, second.data)
        sub.unsubscribe()
        first = topic.get()
        second = topic.get()
        self.assertEqual(first.data+1, second.data)
