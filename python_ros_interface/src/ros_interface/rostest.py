#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=no-member

import os
import random
import subprocess
import rospy
import rosnode

def _wait_nodes(nodes=None, timeout=5.0):
    if nodes:
        if not isinstance(nodes, list):
            nodes = [nodes]
        timeout_time = rospy.get_rostime() + rospy.Duration(timeout)
        rate = rospy.Rate(10)
        while True:
            if rospy.is_shutdown():
                raise RuntimeError('shutdown')
            if rospy.get_rostime() > timeout_time:
                raise RuntimeError('timeout')
            living_nodes = rosnode.get_node_names()
            if all([node in living_nodes for node in nodes]):
                break
            rate.sleep()

class _Master(object):
    def __init__(self):
        self._roscore = None
        self._roslaunch = None
    def init(self, init_node=True):
        if self._roscore is None:
            # randomiaze port for parallel test
            port = random.randrange(10000, 50000)
            os.environ['ROS_MASTER_URI'] = 'http://localhost:%d' % port
            self._roscore = subprocess.Popen(['roscore', '-p', str(port)])
            if not rospy.core.is_initialized() and init_node:
                rospy.init_node('test', anonymous=True)
                _wait_nodes('/rosout')
    def launch(self, launch_file, init_node=True):
        self.init(init_node=init_node)
        self.shutdown()
        if init_node:
            params = rospy.get_param_names()
            for param in params:
                rospy.delete_param(param)
        self._roslaunch = subprocess.Popen(['roslaunch', launch_file])

    def shutdown(self):
        if self._roslaunch:
            self._roslaunch.terminate()
            self._roslaunch.wait()
            self._roslaunch = None
    def __del__(self):
        self.shutdown()
        if self._roscore:
            self._roscore.terminate()
            self._roscore.wait()

_MASTER = _Master()

def rostest_launch(launch_file):
    _MASTER.launch(launch_file)
