#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=no-member

import rospy
from .tf_wrapper import TfWrapper
from .exceptions import *

_TIMEOUT = 5.0

class ROSConnection(object):
    u"""
    Singleton class to manage connection with ROS Network
    """
    _instance = None
    def __new__(cls, **kwargs):
        if ROSConnection._instance is None:
            ROSConnection._instance = super(ROSConnection, cls).__new__(cls, **kwargs)
            if not rospy.core.is_initialized():
                name = cls.__class__.__name__
                rospy.init_node(name.replace('.', '_'), anonymous=True)
            ROSConnection._instance.tf = TfWrapper()
        return ROSConnection._instance

class SubscribeManager(object):
    u"""
    Manager class for subscribers

    Args:
        subscribe_tf:

    Attributes:
        tf:
    """
    def __init__(self, subscribe_tf=True):
        connection = ROSConnection()
        self.tf = connection.tf
        self._subscribe_tf = subscribe_tf
        self._subscribers = []

    def add_subscriber(self, topic_wrapper, wait_first=True, timeout=_TIMEOUT, **kwargs):
        u"""
        Register a subscriber in :attr:`_subscribers`

        Args:
            topic_wrapper:
            wait_first:
            timeout:
            **kwargs:
        """
        kwargs['wait_first'] = wait_first
        kwargs['timeout'] = timeout
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
