#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from .tf2_wrapper import Tf2Wrapper

class _ROSConnection(object):
    u"""
    Singleton class to manage connection with ROS Network

    Attributes:
        tf:
    """
    def __init__(self):
        self.tf = Tf2Wrapper()

    def init(self):
        u"""
        Initialize
        """
        if not rospy.core.is_initialized():
            rospy.init_node('ros_interface', anonymous=True)
        self.tf.init()

_ROS_CONNECTION = _ROSConnection() # Singleton

class SubscribeManager(object):
    u"""
    Manager class for subscribers

    Args:
        subscribe_tf:

    Attributes:
        tf:
    """
    def __init__(self, subscribe_tf=True):
        _ROS_CONNECTION.init()
        self.tf = _ROS_CONNECTION.tf
        self._subscribe_tf = subscribe_tf
        self._subscribers = []

    def add_subscriber(self, topic_wrapper, wait_first=True, **kwargs):
        u"""
        Register a subscriber in :attr:`_subscribers`

        Args:
            topic_wrapper:
            wait_first:
            **kwargs:
        """
        kwargs['wait_first'] = wait_first
        self._subscribers.append((topic_wrapper, kwargs))

    def subscribe(self):
        u"""
        Start subscription
        """
        if self._subscribe_tf:
            self.tf.subscribe()
        for topic_wrapper, kwargs in self._subscribers:
            topic_wrapper.subscribe(**kwargs)

    def unsubscribe(self):
        u"""
        Stop subscription
        """
        # Not stop the tf because other clients may still use it
        for topic_wrapper, _ in self._subscribers:
            topic_wrapper.unsubscribe()
