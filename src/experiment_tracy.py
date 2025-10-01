#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from time import sleep

# General Imports
import numpy as np
import time
from typing import List, Tuple
from copy import deepcopy

# ROS Imports
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from std_msgs.msg import ColorRGBA

# Giskard Imports
from giskardpy.utils.math import quaternion_from_rotation_matrix
from giskardpy_ros.python_interface.python_interface import GiskardWrapper


# Default joint positions for the Tracy robot
default_pose_joints = {
        'left_shoulder_pan_joint': 2.539670467376709,
        'left_shoulder_lift_joint': -1.46823854119096,
        'left_elbow_joint': 2.1197431723224085,
        'left_wrist_1_joint': -1.4825000625899811,
        'left_wrist_2_joint': 5.467689037322998,
        'left_wrist_3_joint': -0.9808381239520472,
        'right_shoulder_pan_joint': 3.7588136196136475,
        'right_shoulder_lift_joint': -1.7489210567870082,
        'right_elbow_joint': -2.054229259490967,
        'right_wrist_1_joint': -1.6140786610045375,
        'right_wrist_2_joint': 0.7295855283737183,
        'right_wrist_3_joint': 0.4919312793523852,
    }

default_l_pose_cartesian = PoseStamped()
default_l_pose_cartesian.header.frame_id = 'table'
default_l_pose_cartesian.pose.position.x = 0.729
default_l_pose_cartesian.pose.position.y = 0.255
default_l_pose_cartesian.pose.position.z = 0.269
default_l_pose_cartesian.pose.orientation = Quaternion(*quaternion_from_rotation_matrix([[-1, 0, 0, 0],
                                                                                         [0, 1, 0, 0],
                                                                                         [0, 0, -1, 0],
                                                                                         [0, 0, 0, 1]]))
default_r_pose_cartesian = PoseStamped()
default_r_pose_cartesian.header.frame_id = 'table'
default_r_pose_cartesian.pose.position.x = 0.788
default_r_pose_cartesian.pose.position.y = -0.239
default_r_pose_cartesian.pose.position.z = 0.295
default_r_pose_cartesian.pose.orientation = Quaternion(*quaternion_from_rotation_matrix([[-1, 0, 0, 0],
                                                                                         [0, 1, 0, 0],
                                                                                         [0, 0, -1, 0],
                                                                                         [0, 0, 0, 1]]))

# Colors for visualization
# Color1 (RGBA: red, fully opaque)
color_red = ColorRGBA(234 / 255.0, 79 / 255.0, 61 / 255.0, 1.0)
# Color2 (RGBA: lime green, fully opaque)
color_lime = ColorRGBA(198 / 255.0, 224 / 255.0, 112 / 255.0, 1.0)


def choose_gripper(pose, l_gripper_pose, r_gripper_pose, dual_heuristic: bool = False) -> Tuple[str, PoseStamped]:
    """
    Choose the gripper based on the distance to the target pose.
    If dual_heuristic is True, it will choose the gripper that is closer to the target pose.
    Otherwise, it will use the left gripper by default.

    :param pose: The target pose to reach.
    :param l_gripper_pose: The pose of the left gripper.
    :param r_gripper_pose: The pose of the right gripper.
    :param dual_heuristic: If True, use the heuristic to choose the gripper
    :return: A tuple containing the gripper name and the default pose of the chosen gripper to return to.
    """
    if dual_heuristic:
        l_distance = np.linalg.norm(
            np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]) -
            np.array([l_gripper_pose.pose.position.x, l_gripper_pose.pose.position.y, l_gripper_pose.pose.position.z]))
        r_distance = np.linalg.norm(
            np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]) -
            np.array([r_gripper_pose.pose.position.x, r_gripper_pose.pose.position.y, r_gripper_pose.pose.position.z])
        )
        if l_distance <= r_distance:
            return "l_gripper_tool_frame", l_gripper_pose
        else:
            return "r_gripper_tool_frame", r_gripper_pose
    else:
        return "l_gripper_tool_frame", l_gripper_pose


def run_experiment(dual_heuristic: bool = False):
    """
Run the dual-arm experiment on the Tracy robot using Giskard.
    :param dual_heuristic: If True, use the heuristic to choose the gripper based on distance to the target pose.
    :return: A 2D numpy array containing the time taken for each pose in the grid.
    """
    global default_pose_joints, default_l_pose_cartesian, default_r_pose_cartesian, color_red, color_lime

    rospy.init_node('dual_arm_experiment')
    giskard = GiskardWrapper()

    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # Move both arms to default position
    a = 'goal1'
    giskard.motion_goals.add_joint_position(default_pose_joints, name=a, end_condition=a)
    giskard.motion_goals.allow_all_collisions()
    giskard.add_default_end_motion_conditions()
    result = giskard.execute()

    # Define the ranges
    x_min, x_max = 0.6, 1.2  # rows
    y_min, y_max = -0.5, 0.5  # cols

    # Define number of points in each dimension
    num_rows = 15  # number of x values
    num_cols = 25  # number of y values

    # Generate the coordinate arrays
    y_values = np.linspace(y_min, y_max, num_cols)
    x_values = np.linspace(x_min, x_max, num_rows)

    # Clear previous markers
    delete_marker = Marker()
    delete_marker.action = Marker.DELETEALL
    delete_marker.id = 0
    pub.publish(delete_marker)

    # Marker setup
    marker = Marker()
    marker.header.frame_id = "tracy/table"  # Change to your TF frame
    marker.type = Marker.CUBE_LIST
    marker.action = Marker.ADD
    marker.id = 0
    # Marker scale (sphere diameter)
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    marker.colors = [color_lime] * (num_rows * num_cols)

    # Create 2D array of poses
    pose_array = []

    default_gripper = "l_gripper_tool_frame"

    # Iterate through rows first (y values)
    for x in x_values:
        row = []
        # Then iterate through columns (x values)
        for y in y_values:
            pose = PoseStamped()
            pose.header.frame_id = 'tracy/table'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.1
            # Set orientation to identity quaternion
            pose.pose.orientation = Quaternion(*quaternion_from_rotation_matrix([[-1, 0, 0, 0],
                                                                                 [0, 1, 0, 0],
                                                                                 [0, 0, -1, 0],
                                                                                 [0, 0, 0, 1]]))

            # Update marker points
            marker.points.append(Point(x=x, y=y, z=0.1))
            row.append(pose)
        pose_array.append(row)

    marker.header.stamp = rospy.Time.now()
    pub.publish(marker)

    # Move right arm out of the way in case of baseline mode
    if not dual_heuristic:
        parking_pose_r = deepcopy(default_r_pose_cartesian)
        parking_pose_r.pose.position.y = parking_pose_r.pose.position.y - 0.3
        giskard.motion_goals.add_cartesian_pose(parking_pose_r, name='move_away', end_condition="move_away",
                                                tip_link="r_gripper_tool_frame", root_link="map")
        giskard.motion_goals.allow_all_collisions()
        giskard.add_default_end_motion_conditions()
        result = giskard.execute()

    times = []
    time_start = time.time()
    for i, row in enumerate(pose_array):
        time_row = []
        for j, pose in enumerate(row):
            # Add each pose to the Giskard motion goals
            name = f'goal_{i}_{j}'

            # Choose the gripper based on the heuristic or default
            gripper, default_pose_cartesian = choose_gripper(pose, default_l_pose_cartesian, default_r_pose_cartesian,
                                                             dual_heuristic=dual_heuristic)

            marker.colors[0] = color_red  # Change color of current target to red
            pub.publish(marker)

            # Move selected gripper to the target pose
            giskard.motion_goals.add_cartesian_pose(pose, name=name, end_condition=name,
                                                    tip_link=gripper, root_link="map")
            giskard.motion_goals.allow_all_collisions()
            giskard.add_default_end_motion_conditions()
            t = time.time()
            result = giskard.execute()
            print(time.time() - t)
            time_row.append(len(result.trajectory.points))

            # Delete currently grasped marker
            marker.points.pop(0)
            marker.colors.pop(0)
            pub.publish(marker)

            # Move gripper back to default pose
            giskard.motion_goals.add_cartesian_pose(default_pose_cartesian, name=name, end_condition=name,
                                                    tip_link=gripper, root_link="map")
            giskard.motion_goals.allow_all_collisions()
            giskard.add_default_end_motion_conditions()
            result = giskard.execute()
        times.append(time_row)

    time_end = time.time()

    print(f'Total time for all poses: {time_end - time_start} seconds')

    times_2d = np.array(times)
    return times_2d


def run_experiment_random(num_random_points: int = 100) -> Tuple[List[tuple], List[tuple]]:
    """Run the dual-arm experiment on the Tracy robot using Giskard with random poses. Running
    both experiments baseline and heuristic mode in one go to ensure same random poses.
    :param num_random_points: Number of random points to generate in the workspace.
    :return: A tuple containing two lists of motion records for baseline and heuristic modes.
    """

    global default_pose_joints, default_l_pose_cartesian, default_r_pose_cartesian, color_red, color_lime

    rospy.init_node('dual_arm_experiment')
    giskard = GiskardWrapper()

    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # Clear previous markers
    delete_marker = Marker()
    delete_marker.action = Marker.DELETEALL
    delete_marker.id = 0
    pub.publish(delete_marker)

    # Define the workspace rectangle
    x_min, x_max = 0.6, 1.2
    y_min, y_max = -0.5, 0.5
    z_position = 0.1  # Fixed height for all positions

    # Random mode - new functionality
    # np.random.seed(42)  # Set seed for reproducibility (optional)

    # Generate random positions within the rectangle
    random_positions = []
    for _ in range(num_random_points):
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)

        pose_1 = PoseStamped()
        pose_1.header.frame_id = 'table'
        pose_1.pose.position.x = x
        pose_1.pose.position.y = y
        pose_1.pose.position.z = z_position
        pose_1.pose.orientation = Quaternion(*quaternion_from_rotation_matrix([[-1, 0, 0, 0],
                                                                             [0, 1, 0, 0],
                                                                             [0, 0, -1, 0],
                                                                             [0, 0, 0, 1]]))

        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)

        pose_2 = PoseStamped()
        pose_2.header.frame_id = 'table'
        pose_2.pose.position.x = x
        pose_2.pose.position.y = y
        pose_2.pose.position.z = z_position
        pose_2.pose.orientation = Quaternion(*quaternion_from_rotation_matrix([[-1, 0, 0, 0],
                                                                               [0, 1, 0, 0],
                                                                               [0, 0, -1, 0],
                                                                               [0, 0, 0, 1]]))

        random_positions.append([pose_1, pose_2])

    # Execute random movements and record times
    motion_records_baseline = []  # List of tuples: ((x, y, z), time, gripper_used)
    motion_records_heuristic = []  # List of tuples: ((x, y, z), time, gripper_used)

    heuristic_usage = [False, True]  # Run baseline first and then with heuristic

    for use_heuristic in heuristic_usage:
        if use_heuristic:
            print('heuristic mode')
        else:
            print('baseline mode')

        last_l_pose_cartesian = default_l_pose_cartesian
        last_r_pose_cartesian = default_r_pose_cartesian

        # Move both arms to default position
        a = 'goal1'
        giskard.motion_goals.add_joint_position(default_pose_joints, name=a, end_condition=a)
        giskard.motion_goals.allow_all_collisions()
        giskard.add_default_end_motion_conditions()
        result = giskard.execute()

        # Move right arm out of the way in case of baseline mode
        if not use_heuristic:
            parking_pose_r = deepcopy(default_r_pose_cartesian)
            parking_pose_r.pose.position.y = parking_pose_r.pose.position.y - 0.3
            giskard.motion_goals.add_cartesian_pose(parking_pose_r, name='move_away', end_condition="move_away",
                                                    tip_link="r_gripper_tool_frame", root_link="map")
            giskard.motion_goals.allow_all_collisions()
            giskard.add_default_end_motion_conditions()
            result = giskard.execute()

        for idx, pose_list in enumerate(random_positions):
            # Move robot to each random pose
            name = f'random_goal_{idx}'

            pose = pose_list[0]

            # Marker setup
            marker = Marker()
            marker.header.frame_id = "tracy/table"  # Change to your TF frame
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            marker.id = 0
            # Marker scale (sphere diameter)
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.colors = [color_red, color_lime]

            marker.points.append(Point(x=pose_list[0].pose.position.x, y=pose_list[0].pose.position.y, z=pose_list[0].pose.position.z))
            marker.points.append(Point(x=pose_list[1].pose.position.x, y=pose_list[1].pose.position.y, z=pose_list[1].pose.position.z))

            marker.header.stamp = rospy.Time.now()
            pub.publish(marker)

            gripper, default_pose_cartesian = choose_gripper(pose, last_l_pose_cartesian, last_r_pose_cartesian,
                                                             dual_heuristic=use_heuristic)

            giskard.motion_goals.add_cartesian_pose(pose, name=name, end_condition=name,
                                                    tip_link=gripper, root_link="map")
            giskard.motion_goals.allow_all_collisions()
            giskard.add_default_end_motion_conditions()

            t = time.time()
            result = giskard.execute()
            print(time.time() - t)
            if not use_heuristic:
                motion_records_baseline.append(((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
                                                len(result.trajectory.points), gripper))
            else:
                motion_records_heuristic.append(((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
                                                 len(result.trajectory.points), gripper))

            # Return to random drop-off position
            pose_drop_off = pose_list[1]
            giskard.motion_goals.add_cartesian_pose(pose_drop_off, name=name, end_condition=name,
                                                    tip_link=gripper, root_link="map")
            giskard.motion_goals.allow_all_collisions()
            giskard.add_default_end_motion_conditions()
            result = giskard.execute()

            if gripper == 'l_gripper_tool_frame':
                last_l_pose_cartesian = pose_drop_off
            elif gripper == 'r_gripper_tool_frame':
                last_r_pose_cartesian = pose_drop_off
            else:
                raise ValueError("Wrong gripper type")

        sleep(10)  # Wait before starting the next experiment

    return motion_records_baseline, motion_records_heuristic
