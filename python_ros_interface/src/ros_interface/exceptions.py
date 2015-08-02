#!/usr/bin/env python
# -*- coding: utf-8 -*-
u"""
Provides simplified ROS Python interface
"""

class ROSInterfaceException(Exception):
    pass
class ROSInterfaceRuntimeError(RuntimeError):
    pass
class TimeoutException(ROSInterfaceRuntimeError):
    pass
class NotResolvableException(ROSInterfaceRuntimeError):
    pass
