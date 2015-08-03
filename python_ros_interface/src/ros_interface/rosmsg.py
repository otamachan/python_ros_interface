#!/usr/bin/env python
# -*- coding: utf-8 -*-
u"""
Utilities for ROS messages
"""
import copy
import numpy
import rospy
import genpy
import tf.transformations
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import trajectory_msgs.msg

def TransformStamped(transform, parent_id='', child_id='', stamp=None):
    u"""
    Return :class:`geometry_msgs.msg.TransformStapmped`

    Args:
        transform:
        stamp:
    Return:
        Stamped object
    Example:
        .. code-block:: python

            print rosmsg.Stamped(rosmsg.Pose(x=1), frame_id='aa')
    """

def Stamped(obj, frame_id='', stamp=None):
    u"""
    Return stamped ( with :class:`std_msgs.msg.Header` ) object

    Args:
        frame_id:
        stamp:
    Return:
        Stamped object
    Example:
        .. code-block:: python

            print rosmsg.Stamped(rosmsg.Pose(x=1), frame_id='aa')
    """
    package_name = obj.__module__.split('.', 1)[0]
    message_type = obj.__class__.__name__ + 'Stamped'
    cls = genpy.message.get_message_class(package_name + '/' + message_type)
    if stamp is None:
        stamp = rospy.Time.now()
    return cls(std_msgs.msg.Header(frame_id=frame_id, stamp=stamp),
               obj)

def JointTrajectoryPoint(positions=None, duration=0.0):
    u"""
    Create :class:`trajectory_msgs.msg.JointTrajectoryPoint`

    Args:
        positions:
    Return:
        :class:`trajectory_msgs.msg.JointTrajectoryPoint`
    Example:
        .. code-block:: python

            print rosmsg.JointTrajectoryPoint({'larm_elbow': 11.0})
    """
    if isinstance(positions, list):
        zero = [0.0] * len(positions)
        return trajectory_msgs.msg.JointTrajectoryPoint(
            positions=positions,
            velocities=zero,
            accelerations=zero,
            time_from_start=rospy.Duration(duration))
    else:
        raise ValueError("positions is invalid: {0}".format(positions))

def JointState(positions=None):
    u"""
    Create :class:`sensor_msgs.msg.JointState`

    Args:
        joint_state:
    Return:
        :class:`sensor_msgs.msg.JointState`
    Example:
        .. code-block:: python

            print rosmsg.JointState({'larm_elbow': 11.0})
    """
    if isinstance(positions, dict):
        return sensor_msgs.msg.JointState(name=positions.keys(),
                                          position=positions.values())
    else:
        raise ValueError("positions is invalid: {0}".format(positions))

def Accel(x=0.0, y=0.0, z=0.0, linear=None,
          ax=0.0, ay=0.0, az=0.0, angular=None):
    pass

def Wrench(x=0.0, y=0.0, z=0.0, linear=None,
           ax=0.0, ay=0.0, az=0.0, angular=None):
    pass

def Twist(x=0.0, y=0.0, z=0.0, linear=None,
          ax=0.0, ay=0.0, az=0.0, angular=None):
    u"""
    Create :class:`geometry_msgs.msg.Twist`

    Args:
        x: (default: 0.0)
        y: (default: 0.0)
        z: (default: 0.0)
        linear: (default: None)
        ax: (default: 0.0)
        ay: (default: 0.0)
        az: (default: 0.0)
        angular: (default: None)
    Return:
        :class:`geometry_msgs.msg.Twist`
    Example:
        .. code-block:: python

            print rosmsg.Twist(x=1)

            print rosmsg.Twist(linear=(3, 1, 2))
            print rosmsg.Twist(linear=Vector3(1, 1, 1))

            print rosmsg.Twist(az=1)

            print rosmsg.Twist(angular=(3, 1, 2))
            print rosmsg.Twist(angular=Vector3(0.1, 0.2, 0.3))
    """
    return geometry_msgs.msg.Twist(Vector3(x, y, z, linear),
                                   Vector3(ax, ay, az, angular))

def _vector3_array(vector3):
    u"""
    Create :class:`numpy.arrrany` from :class:``geometry_msgs.msg.Vector3``
    """
    return numpy.array((vector3.x, vector3.y, vector3.z))

def _quaternion_array(quaternion):
    u"""
    Create :class:`numpy.arrray` from :class:``geometry_msgs.msg.Quaternion``
    """
    return numpy.array((quaternion.x,
                        quaternion.y,
                        quaternion.z,
                        quaternion.w))

def _multiply_transforms(t1, t2):
    u"""
    Multiply :class:`geometry_msgs.msg.Transform`
    """
    trans1_mat = tf.transformations.translation_matrix(_vector3_array(t1.translation))
    rot1_mat = tf.transformations.quaternion_matrix(_quaternion_array(t1.rotation))
    mat1 = numpy.dot(trans1_mat, rot1_mat)

    trans2_mat = tf.transformations.translation_matrix(_vector3_array(t2.translation))
    rot2_mat = tf.transformations.quaternion_matrix(_quaternion_array(t2.rotation))
    mat2 = numpy.dot(trans2_mat, rot2_mat)

    mat3 = numpy.dot(mat1, mat2)
    trans3 = tf.transformations.translation_from_matrix(mat3)
    rot3 = tf.transformations.quaternion_from_matrix(mat3)

    return Transform(translation=trans3,
                     rotation=rot3)

def _inverse_transform(t):
    u"""
    Inverse :class:`geometry_msgs.msg.Transform`
    """
    inv_trans_mat = tf.transformations.translation_matrix(-1.0*_vector3_array(t.translation))
    inv_rot = tf.transformations.quaternion_inverse(_quaternion_array(t.rotation))
    inv_rot_mat = tf.transformations.quaternion_matrix(inv_rot)
    mat = numpy.dot(inv_rot_mat, inv_trans_mat)
    inv_trans = tf.transformations.translation_from_matrix(mat)
    return Transform(translation=inv_trans,
                     rotation=inv_rot)

def Transform(x=0.0, y=0.0, z=0.0, translation=None,
              ai=0.0, aj=0.0, ak=0.0, axes='sxyz', rotation=None,
              world=None, local=None,
              inverse=None):
    u"""
    Create :class:`geometry_msgs.msg.Transform`

    Args:
        x: (default: 0.0)
        y: (default: 0.0)
        z: (default: 0.0)
        translation: (default: None)
        ai: (default: 0.0)
        aj: (default: 0.0)
        ak: (default: 0.0)
        axes: (default: 'sxyz')
        rotation: (default: None)
        world: (default: None)
        local: (default: None)
        inverse: (default: None)
    Return:
        :class:`geometry_msgs.msg.Transform`
    Example:
        .. code-block:: python

            print rosmsg.Transform(x=1)

            print rosmsg.Transform(translation=(3, 1, 2))
            print rosmsg.Transform(translation=Vector3(3, 1, 2))

            print rosmsg.Transform(ak=0.1)

            print rosmsg.Transform(rotation=(0, 0, 0, 1))
            print rosmsg.Transform(rotation=Quaternion(0, 0, 0, 1))

            parent = rosmsg.Transform(x=1)
            child = rosmsg.Transform(aj=0.2)
            print rosmsg.Transform(world=parent, local=child)

            trans = rosmsg.Transform(x=1, aj=0.1)
            print rosmsg.Transform(inverse=trans)
    """
    if (world is None) != (local is None):
        raise ValueError("world, local should be set both")
    if world:
        return _multiply_transforms(world, local)
    elif inverse:
        return _inverse_transform(inverse)
    return geometry_msgs.msg.Transform(Vector3(x, y, z, translation),
                                       Quaternion(ai, aj, ak, axes, rotation))

def Pose(x=0.0, y=0.0, z=0.0, position=None,
         ai=0.0, aj=0.0, ak=0.0, axes='sxyz', orientation=None,
         transform=None):
    u"""
    Create :class:`geometry_msgs.msg.Pose`

    Args:
        x: (default: 0.0)
        y: (default: 0.0)
        z: (default: 0.0)
        position: (default: None)
        ai: (default: 0.0)
        aj: (default: 0.0)
        ak: (default: 0.0)
        axes: (default: 'sxyz')
        orientation: (default: None)
        transform: (default: None)
    Return:
        :class:`geometry_msgs.msg.Pose`
    Example:
        .. code-block:: python

            print rosmsg.Pose(x=1)

            print rosmsg.Pose(position=(3, 1, 2))
            print rosmsg.Pose(position=Vector3(3, 1, 2))

            print rosmsg.Pose(ak=0.1)

            print rosmsg.Pose(orientation=(0, 0, 0, 1))
            print rosmsg.Pose(orientation=Quaternion(0, 0, 0, 1))

            trans = rosmsg.Transform(x=1, aj=0.1)
            print rosmsg.Pose(transform=trans)
    """
    if isinstance(transform, geometry_msgs.msg.Transform):
        return geometry_msgs.msg.Pose(transform.translation,
                                      transform.rotation)
    elif transform:
        raise ValueError("transform is not {0}".format(geometry_msgs.msg.Transform))
    return geometry_msgs.msg.Pose(Point(x, y, z, position),
                                  Quaternion(ai, aj, ak, axes, orientation))

def Quaternion(ai=0.0, aj=0.0, ak=0.0, axes='sxyz',
               quaternion=None):
    u"""
    Create :class:`geometry_msgs.msg.Quaternion`

    Args:
        ai: (default: 0.0)
        aj: (default: 0.0)
        ak: (default: 0.0)
        axes: (default: 'sxyz')
        quaternion: (default: None)
    Return:
        :class:`geometry_msgs.msg.Quaternion`
    Example:
        .. code-block:: python

            print rosmsg.Quaternion(ak=0.1)

            print rosmsg.Quaternion(ai=0.1, axes='rxyz')

            print rosmsg.Quaternion(quatrenion=(0, 0, 0, 1))
    """
    if quaternion is not None:
        if isinstance(quaternion, geometry_msgs.msg.Quaternion):
            quaternion = copy.deepcopy(quaternion)
        elif len(quaternion) == 4:
            quaternion = geometry_msgs.msg.Quaternion(quaternion[0],
                                                      quaternion[1],
                                                      quaternion[2],
                                                      quaternion[3])
        else:
            raise ValueError("quaternion is invalid: {0}".format(quaternion))
    else:
        array = tf.transformations.quaternion_from_euler(ai, aj, ak, axes=axes)
        quaternion = geometry_msgs.msg.Quaternion(*array)
    return quaternion

def Vector3(x=0.0, y=0.0, z=0.0, vector3=None):
    u"""
    Create :class:`geometry_msgs.msg.Vector3`

    Args:
        x: (default: 0.0)
        y: (default: 0.0)
        z: (default: 0.0)
        vector3: (default: None)
    Return:
        :class:`geometry_msgs.msg.Vector3`
    Example:
        .. code-block:: python

            print rosmsg.Vector3(x=0.1)

            print rosmsg.Vector3(vector3=(0, 0, 0, 1))
    """
    if vector3 is not None:
        if isinstance(vector3, geometry_msgs.msg.Vector3):
            vector3 = copy.deepcopy(vector3)
        elif len(vector3) == 3:
            vector3 = geometry_msgs.msg.Vector3(vector3[0],
                                                vector3[1],
                                                vector3[2])
        else:
            raise ValueError("vector3 is invalid: {0}".format(vector3))
    else:
        vector3 = geometry_msgs.msg.Vector3(x, y, z)
    return vector3

def Point(x=0.0, y=0.0, z=0.0, point=None):
    u"""
    Create :class:`geometry_msgs.msg.Quaternion`

    Args:
        x: (default: 0.0)
        y: (default: 0.0)
        z: (default: 0.0)
        point: (default: None)
    Return:
        :class:`geometry_msgs.msg.Point`
    Example:
        .. code-block:: python

            print rosmsg.Point(x=0.1)

            print rosmsg.Point(point=(0, 0, 1))
    """
    if point is not None:
        if isinstance(point, geometry_msgs.msg.Point):
            pass
        elif len(point) == 3:
            point = geometry_msgs.msg.Point(point[0],
                                            point[1],
                                            point[2])
        else:
            raise ValueError("point is invalid: {0}".format(point))
    else:
        point = geometry_msgs.msg.Point(x, y, z)
    return point
