python_ros_interface's documentation
====================================

Introdction
-----------

**python_ros_interface** provides simplified ROS Python interfaces.

Using rospy directly:

  .. code-block:: python

     import rospy
     from beginner_tutorials.srv import *

     def add_two_ints_client(x, y):
         rospy.wait_for_service('add_two_ints')
         try:
             add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
             resp1 = add_two_ints(x, y)
             return resp1.sum
         except rospy.ServiceException, e:
             print "Service call failed: %s"%e

     print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))

Using ros_interface:

  .. code-block:: python

     from ros_interface import ROSService

     add_two_ints = ROSService('/add_two_ints')
     print "%s + %s = %s"%(x, y, add_two_ints(x, y).sum)

.. toctree::
   :maxdepth: 2
   :numbered:
   :hidden:

   examples
   references
   indexes
