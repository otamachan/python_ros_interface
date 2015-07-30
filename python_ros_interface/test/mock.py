#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rospy_tutorials.srv import AddTwoInts, AddTwoIntsResponse

class Mock(object):
    def __init__(self):
        self.service = rospy.Service('add_two_ints', AddTwoInts, self.add_two_ints)

    def add_two_ints(self, req):
        rospy.logdebug('add_two_ints called')
        return AddTwoIntsResponse(req.a + req.b)

if __name__ == "__main__":
    rospy.init_node('mock')
    m = Mock()
    rospy.spin()
