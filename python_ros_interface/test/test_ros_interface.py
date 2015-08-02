#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import rostest
import logging
import rospy
import std_msgs
from rospy_tutorials.srv import AddTwoIntsRequest
from rospy import ROSException
from ros_interface import ROSService, ROSAction, ROSTopic, ROSParam
from ros_interface import ROSInterfaceRuntimeError

class TestROSInterface(unittest.TestCase):
    # Test ROSService
    def test_rosservice_success(self):
        add_two_ints = ROSService('/add_two_ints')
        self.assertEqual(add_two_ints(1, 2).sum, 3)

    def test_rosservice_get_request(self):
        add_two_ints = ROSService('/add_two_ints')
        self.assertIs(add_two_ints.request, AddTwoIntsRequest)

    def test_rosservice_not_resolvable(self):
        with self.assertRaises(ROSException):
            add_two_ints = ROSService('/add_two_ints_2')
            start = rospy.Time.now()
            add_two_ints(3, 4)
        self.assertGreater(rospy.Time.now() - start, rospy.Duration(1.0))

    def test_rosservice_timeout(self):
        with self.assertRaises(ROSException):
            add_two_ints = ROSService('/add_two_ints_2', timeout=0.5)
            start = rospy.Time.now()
            add_two_ints(3, 4)
        self.assertLess(rospy.Time.now() - start, rospy.Duration(1.0))

    # Test ROSAction
    def test_rosaction_success(self):
        fibonacci = ROSAction('/fibonacci')
        self.assertEqual(fibonacci(5).sequence, (0, 1, 1, 2, 3, 5))
        self.assertEqual(fibonacci.get_state(), fibonacci.SUCCEEDED)
        self.assertIn(fibonacci.get_state(), fibonacci.TERMINAL)

    def test_rosaction_not_resolvable(self):
        fibonacci = ROSAction('/fibonacci_2')
        with self.assertRaises(ROSInterfaceRuntimeError):
            fibonacci(2)

    def test_rosaction_timouet(self):
        fibonacci = ROSAction('/fibonacci')
        start = rospy.Time.now()
        fibonacci(100, timeout=1.0)
        self.assertLess(rospy.Time.now() - start, rospy.Duration(1.5))
        self.assertEqual(fibonacci.get_state(), fibonacci.PREEMPTED)
        self.assertIn(fibonacci.get_state(), fibonacci.TERMINAL)

    def test_rosaction_send_goal_success(self):
        fibonacci = ROSAction('/fibonacci')
        goal = fibonacci.goal(5)
        fibonacci.send_goal(goal)
        fibonacci.wait_for_result()
        self.assertEqual(fibonacci.get_result().sequence, (0, 1, 1, 2, 3, 5))
        self.assertEqual(fibonacci.get_state(), fibonacci.SUCCEEDED)
        self.assertIn(fibonacci.get_state(), fibonacci.TERMINAL)

    def test_rosaction_send_goal_timeout(self):
        fibonacci = ROSAction('/fibonacci')
        goal = fibonacci.goal(100)
        fibonacci.send_goal(goal)
        start = rospy.Time.now()
        fibonacci.wait_for_result(rospy.Duration(1.0))
        self.assertLess(rospy.Time.now() - start, rospy.Duration(1.5))
        self.assertEqual(fibonacci.get_state(), fibonacci.ACTIVE)
        self.assertNotIn(fibonacci.get_state(), fibonacci.TERMINAL)
        fibonacci.cancel_goal()

    def test_rosaction_with_callback_success(self):
        self._done = False
        self._count = 0
        fibonacci = ROSAction('/fibonacci')
        fibonacci.send_goal(fibonacci.goal(5),
                            feedback_cb=self.action_feedback,
                            done_cb=self.action_done)
        while not self._done:
            rospy.sleep(0.5)
        self.assertEqual(fibonacci.get_state(), fibonacci.SUCCEEDED)
        self.assertEqual(self._count, 4)
        self.assertTrue(self._done)
        self.assertIn(fibonacci.get_state(), fibonacci.TERMINAL)

    def action_done(self, status, result):
        self._done = True

    def action_feedback(self, data):
        self._count += 1

    # Test ROSTopic
    def test_rostopic_get_success(self):
        counter = ROSTopic('/counter_pub')
        first = counter.get()
        second = counter.get()
        self.assertEqual(first.data + 1, second.data)

    def test_rostopic_get_not_resolvable(self):
        counter = ROSTopic('/counter_pub_2')
        with self.assertRaises(ROSInterfaceRuntimeError):
            counter.get()

    def test_rostopc_subscribe_success(self):
        counter = ROSTopic('/counter_pub')
        counter.subscribe()
        first = counter.get()
        second = counter.get()
        self.assertEqual(first.data, second.data)

    def test_rostopic_subscribe_get_wait_update(self):
        counter = ROSTopic('/counter_pub')
        counter.subscribe()
        first = counter.get()
        second = counter.get(wait_update=True)
        self.assertEqual(first.data + 1, second.data)

    def test_rostopic_subscribe_not_resolvable(self):
        counter = ROSTopic('/counter_pub_2')
        with self.assertRaises(ROSInterfaceRuntimeError):
            counter.subscribe()

    def test_rostopic_subscribe_with_callback(self):
        self._counter_data = None
        counter = ROSTopic('/counter_pub')
        counter.subscribe(callback=self.sub_callback)
        while self._counter_data is None:
            rospy.sleep(0.5)
        first = self._counter_data
        while self._counter_data == first:
            rospy.sleep(0.5)
        second = self._counter_data
        self.assertEqual(first.data + 1, second.data)

    def sub_callback(self, data):
        self._counter_data = data

    def test_rostopic_put_success(self):
        counter = ROSTopic('/counter_sub')
        counter.put(3)
        self.assertEqual(ROSTopic('/counter_echo').get().data, 4)
    def test_rostopic_put_not_resolvable(self):
        with self.assertRaises(ROSInterfaceRuntimeError):
            counter = ROSTopic('/counter_sub_2')
            counter.put(3)

    def test_rostopic_put_not_wait_for_subscribers(self):
        counter = ROSTopic('/counter_sub', wait_for_subscribers=False, latch=True)
        counter.put(3)
        self.assertEqual(ROSTopic('/counter_echo').get().data, 4)

    def test_rostopic_put_with_data_class(self):
        counter = ROSTopic('/counter_sub_2', wait_for_subscribers=False, data_class=std_msgs.msg.Int32)
        counter.put(3)

    def test_rostopic_put_timeout(self):
        counter = ROSTopic('/counter_sub_2', publisher_timeout=0.5, data_class=std_msgs.msg.Int32)
        with self.assertRaises(ROSInterfaceRuntimeError):
            start = rospy.Time.now()
            counter.put(3)
        self.assertLess(rospy.Time.now() - start, rospy.Duration(1.0))

class TestROSParam(unittest.TestCase):
    def test_get(self):
        pass
    def test_get_once(self):
        pass
    def test_put(self):
        pass

if __name__ == '__main__':
    import rostest
    rospy.init_node('test_ros_interface')
    logging.getLogger('rosout').setLevel(logging.DEBUG)
    rostest.rosrun('python_ros_interface', 'testros_interface', TestROSInterface)
