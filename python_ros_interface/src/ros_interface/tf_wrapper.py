#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=no-member

import rospy
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class TfWrapper(object):
    u"""
    tf2_ros wrapper class

    Args:
        timeout:

    Attributes:
        timeout:
    """
    def __init__(self, timeout=1.0):
        self._tf2_buffer = None
        self._tf2_listener = None
        self.timeout = timeout
        self._tf_publisher = None
    def init(self):
        if self._tf2_buffer is None:
            self._tf2_buffer = tf2_ros.Buffer()
            self._tf_publisher = rospy.Publisher("/tf", TFMessage, queue_size=1)
    def subscribe(self):
        u"""
        Start subscription from tf
        """
        self._tf2_listener = tf2_ros.TransformListener(self._tf2_buffer)
    def unsubscribe(self):
        u"""
        Stop subscription from tf
        """
        self._tf2_listener = None
    def send_one_transform(self, parent, child, transform, stamp=None):
        u"""
        Publish a single transform to tf
        """
        if stamp is None:
            stamp = rospy.Time.now()
        self.send_transform([TransformStamped(rospy.Header(frame_id=parent, stamp=stamp),
                                              child,
                                              transform)])

    def send_transform(self, transform_stamped):
        u"""
        Publish transforms
        """
        if not isinstance(transform_stamped, list):
            transform_stamped = [transform_stamped]
        self._tf_publisher.publish(TFMessage(transform_stamped))

    def lookup_transform(self, target_frame, source_frame, time=None, timeout=None):
        u"""
        return :class:`geometry_msgs.msg.Transform`

        Args:
            target_frame:
            source_frame:
            time:
            timeout:
        Return:
            :class:`geometry_msgs.Transform`
        """
        if time is None:
            time = rospy.Time(0)
        if timeout is None:
            timeout = rospy.Duration(self.timeout)
        transform_stamped = self._tf2_buffer.lookup_transform(target_frame,
                                                              source_frame,
                                                              time,
                                                              timeout)
        return transform_stamped.transform
