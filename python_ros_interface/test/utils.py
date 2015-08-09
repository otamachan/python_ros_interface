#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=no-member

import subprocess
import rospy
import rosnode

_MASTER = None

class Master(object):
    def __init__(self):
        self.proc = subprocess.Popen(['roscore'])
        rospy.init_node('test', anonymous=True)
    def __del__(self):
        self.proc.terminate()
        self.proc.wait()

def launch(filename, wait_nodes=None, timeout=5.0):
    global _MASTER
    if _MASTER is None:
        _MASTER = Master()
    # clear all params
    params = rospy.get_param_names()
    for param in params:
        rospy.delete_param(param)
    # launch
    proc = subprocess.Popen(['roslaunch', filename])
    # wait nodes
    if wait_nodes:
        if not isinstance(wait_nodes, list):
            wait_nodes = [wait_nodes]
        timeout_time = rospy.get_rostime() + rospy.Duration(timeout)
        rate = rospy.Rate(10)
        while True:
            if rospy.is_shutdown():
                raise RuntimeError()
            if timeout_time and rospy.get_rostime() > timeout_time:
                raise RuntimeError()
            nodes = rosnode.get_node_names()
            if all([node in wait_nodes for node in nodes]):
                break
            rate.sleep()
    return proc

def wait_shutdown(proc):
    proc.terminate()
    proc.wait()
