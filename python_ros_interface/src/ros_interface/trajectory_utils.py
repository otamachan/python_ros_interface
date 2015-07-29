#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def remove_invalid(traj):
    u"""
    Remove invalid ``JointTrajectory``

    Args:
        traj:
    Retrun:
        :class:`JointTrajectory`
    """
    valid_indices = set(range(len(traj.joint_names)))
    if len(traj.points) > 0:
        nan_velocity_index = (index for index, velocity in enumerate(traj.points[0].velocities)
                              if math.isnan(velocity))
        valid_indices.difference_update(nan_velocity_index)
    new_joint_trajectory = JointTrajectory(joint_names=[traj.joint_names[i] for i in valid_indices])
    for point in traj.points:
        new_point = JointTrajectoryPoint(positions=tuple((point.positions[i] for i in valid_indices)),
                                         velocities=tuple((point.velocities[i] for i in valid_indices)),
                                         accelerations=tuple((point.accelerations[i] for i in valid_indices)),
                                         effort=[],
                                         time_from_start=point.time_from_start)
        new_joint_trajectory.points.append(new_point)
    return new_joint_trajectory

def add(traj1, traj2):
    u"""
    Add two :class:`JointTrajectory` s

    Args:
        traj1:
        traj2:
    Retrun:
        :class:`JointTrajectory`
    """
    new_joint_trajectory = JointTrajectory(joint_names=traj1.joint_names+traj2.joint_names)
    for point1, point2 in zip(traj1.points, traj2.points):
        new_point = JointTrajectoryPoint(positions=tuple(point1.positions)+tuple(point2.positions),
                                         velocities=tuple(point1.velocities)+tuple(point2.velocities),
                                         accelerations=tuple(point1.accelerations)+tuple(point2.accelerations),
                                         effort=[],
                                         time_from_start=point1.time_from_start)
        new_joint_trajectory.points.append(new_point)
    return new_joint_trajectory

def extract(joint_names, joint_trajectory):
    u"""
    Extract joint_trajecroy

    Args:
        joint_names:
        joint_trajectory:
    Retrun:
        :class:`JointTrajectory`
    Raises:
        RuntimeError:
    """
    out_joint_names = [joint_name for joint_name in joint_names
                       if joint_name in joint_trajectory.joint_names]
    out_joint_trajectory = JointTrajectory(joint_names=out_joint_names)
    # calcurate indexes
    indexes = [joint_trajectory.joint_names.index(joint_name)
               for joint_name in out_joint_names]
    # create points
    for point in joint_trajectory.points:
        out_point = JointTrajectoryPoint(
            positions=[point.positions[index] for index in indexes],
            velocities=[point.velocitiess[index] for index in indexes],
            accelerations=[point.accelerations[index] for index in indexes],
            time_from_start=point.time_from_start)
        out_joint_trajectory.points.append(out_point)
    return out_joint_trajectory
