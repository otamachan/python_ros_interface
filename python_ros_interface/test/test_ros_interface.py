#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=no-member

import unittest
import rostest
import logging
import rospy
import std_msgs
from rospy_tutorials.srv import AddTwoIntsRequest
from rospy import ROSException
from ros_interface import ROSService, ROSAction, ROSTopic, ROSParam
from ros_interface import ROSServiceProp, ROSActionProp, ROSTopicProp, ROSParamProp, ROSInterface
from ros_interface import ROSInterfaceRuntimeError

class MockNode(ROSInterface):
    _properties = {'add_two_ints': ROSServiceProp(),
                   'fibonacci': ROSActionProp(),
                   'counter_pub': ROSTopicProp(),
                   'counter_sub': ROSTopicProp(),
                   'param': ROSParamProp()}

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

    # Test ROSParam
    def test_get_success(self):
        rospy.set_param('/param1', 1)
        param1 = ROSParam('/param1')
        self.assertEqual(param1.get(), 1)

        rospy.set_param('/param2/var', 2)
        param2 = ROSParam('/param2')
        self.assertEqual(param2.get(), {'var': 2})

        var = ROSParam('/param2/var')
        self.assertEqual(var.get(), 2)

    def test_get_from_cache(self):
        rospy.set_param('/param3', 3)
        param3 = ROSParam('/param3')
        self.assertEqual(param3.get(), 3)
        rospy.set_param('/param3', 0)
        self.assertEqual(param3.get(), 3)

    def test_get_cache_off(self):
        rospy.set_param('/param3', 3)
        param3 = ROSParam('/param3', cache=False)
        self.assertEqual(param3.get(), 3)
        rospy.set_param('/param3', 0)
        self.assertEqual(param3.get(), 0)

    def test_get_with_default_value(self):
        param4 = ROSParam('/param4')
        self.assertEqual(param4.get(0), 0)

    def test_get_with_suffix(self):
        param2 = ROSParam('/param2')
        self.assertEqual(param2.get(suffix='var'), 2)

    def test_clear_cache(self):
        rospy.set_param('/param3', 3)
        param3 = ROSParam('/param3')
        self.assertEqual(param3.get(), 3)
        rospy.set_param('/param3', 0)
        param3.clear_cache()
        self.assertEqual(param3.get(), 0)

    def test_set_success(self):
        rospy.set_param('/param1', 0)
        param1 = ROSParam('/param1')
        param1.set(1)
        self.assertEqual(rospy.get_param('/param1'), 1)

        rospy.set_param('/param2/var', 0)
        var = ROSParam('/param2')
        var.set({'var': 3})
        self.assertEqual(rospy.get_param('/param2/var'), 3)

        rospy.set_param('/param2/var', 0)
        var = ROSParam('/param2/var')
        var.set(2)
        self.assertEqual(rospy.get_param('/param2/var'), 2)

    def test_set_cache(self):
        rospy.set_param('/param1', 0)
        param1 = ROSParam('/param1')
        param1.set(1)
        rospy.set_param('/param1', 2)
        self.assertEqual(param1.get(), 1)

    def test_set_no_cache(self):
        rospy.set_param('/param1', 0)
        param1 = ROSParam('/param1', cache=False)
        param1.set(1)
        rospy.set_param('/param1', 2)
        self.assertEqual(param1.get(), 2)

    def test_set_with_suffix(self):
        rospy.set_param('/param2/var', 0)
        param1 = ROSParam('/param2')
        param1.set(2, suffix='var')
        self.assertEqual(rospy.get_param('/param2/var'), 2)

    def test_ros_interface(self):
        mock = MockNode()
        self.assertEqual(mock.add_two_ints(1, 2).sum, 3)
        self.assertEqual(mock.fibonacci(5).sequence, (0, 1, 1, 2, 3, 5))
        first = mock.counter_pub.get()
        second = mock.counter_pub.get()
        self.assertEqual(first.data+1, second.data)
        mock.counter_sub.put(1)
        self.assertEqual(ROSTopic('/counter_echo').get().data, 2)
        self.assertEqual(mock.param, 1)

    def test_ros_interface_with_namespace(self):
        mock = MockNode('/namespace')
        self.assertEqual(mock.add_two_ints(2, 2).sum, 4)
        self.assertEqual(mock.fibonacci(6).sequence, (0, 1, 1, 2, 3, 5, 8))
        first = mock.counter_pub.get()
        second = mock.counter_pub.get()
        self.assertEqual(first.data+1, second.data)
        mock.counter_sub.put(2)
        self.assertEqual(ROSTopic('/namespace/counter_echo').get().data, 3)
        self.assertEqual(mock.param, 2)

if __name__ == '__main__':
    import rostest
    rospy.init_node('test_ros_interface')
    logging.getLogger('rosout').setLevel(logging.DEBUG)
    rostest.rosrun('python_ros_interface', 'testros_interface', TestROSInterface)
