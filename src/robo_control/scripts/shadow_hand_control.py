#!/usr/bin/env python3

# Software License Agreement (BSD License)
# Copyright © 2019, 2022-2023 belongs to Shadow Robot Company Ltd.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#   1. Redistributions of source code must retain the above copyright notice,
#      this list of conditions and the following disclaimer.
#   2. Redistributions in binary form must reproduce the above copyright notice,
#      this list of conditions and the following disclaimer in the documentation
#      and/or other materials provided with the distribution.
#   3. Neither the name of Shadow Robot Company Ltd nor the names of its contributors
#      may be used to endorse or promote products derived from this software without
#      specific prior written permission.
#
# This software is provided by Shadow Robot Company Ltd "as is" and any express
# or implied warranties, including, but not limited to, the implied warranties of
# merchantability and fitness for a particular purpose are disclaimed. In no event
# shall the copyright holder be liable for any direct, indirect, incidental, special,
# exemplary, or consequential damages (including, but not limited to, procurement of
# substitute goods or services; loss of use, data, or profits; or business interruption)
# however caused and on any theory of liability, whether in contract, strict liability,
# or tort (including negligence or otherwise) arising in any way out of the use of this
# software, even if advised of the possibility of such damage.

# Example where two joints are specified and move with a sinusoidal trajectory, with a pi/4 phase difference

import rospy
from numpy import sin, cos, pi, arange
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sr_utilities.hand_finder import HandFinder
from sr_robot_commander.sr_hand_commander import SrHandCommander

import scipy.io as sio
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import numpy.random as random
import math
from typing import Any, Dict
import json
import os

rospy.init_node("joint_sine_example", anonymous=True)

# handfinder is used to access the hand parameters
hand_finder = HandFinder()
hand_parameters = hand_finder.get_hand_parameters()
prefix = list(hand_parameters.mapping.values())
hand_serial = list(hand_parameters.mapping.keys())[0]

hand_commander = SrHandCommander(hand_parameters=hand_parameters, hand_serial=hand_serial)

# cycles per second of sine wave
frequency = 1
# angular frequency, rads/s
angular_frequency = 2 * pi * frequency

# set max and min joint positions
min_pos_j3 = 0.0
max_pos_j3 = pi / 2

rospy.sleep(rospy.Duration(2))

def load_json_to_dict(file_path: str) -> Dict[str, Any]:
    """
    从JSON文件读取数据并返回字典
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as json_file:
            return json.load(json_file)
    except FileNotFoundError:
        print(f"错误: 文件 {file_path} 未找到")
        return {}
    except json.JSONDecodeError as e:
        print(f"错误: JSON解析失败 -> {e}")
        return {}
    except Exception as e:
        print(f"错误: {e}")
        return {}

# 使用示例
joint_angle = load_json_to_dict("/home/user/workspace/src/robo_control/scripts/joint_angle.json")

print(joint_angle)
# joint_angle = {prefix[0] + '_FFJ4': -10.0, prefix[0] + '_FFJ3': 45.0, prefix[0] + '_FFJ2': 45.0, prefix[0] + '_FFJ1': 45.0, \
#             prefix[0] + '_MFJ4': -5.0, prefix[0] + '_MFJ3': 45.0, prefix[0] + '_MFJ2': 45.0, prefix[0] + '_MFJ1': 45.0, \
#             prefix[0] + '_RFJ4': -5.0, prefix[0] + '_RFJ3': 45.0, prefix[0] + '_RFJ2': 45.0, prefix[0] + '_RFJ1': 45.0, \
#             prefix[0] + '_LFJ4': -10.0, prefix[0] + '_LFJ3': 45.0, prefix[0] + '_LFJ2': 45.0, prefix[0] + '_LFJ1': 45.0, \
#             prefix[0] + '_THJ4': 20.0, prefix[0] + '_THJ3': 45.0, prefix[0] + '_THJ2': 45.0, prefix[0] + '_THJ1': 45.0}
# print(joint_angle)
rospy.loginfo("Running joints trajectory")

# initialising the joint trajectory message
joint_trajectory = JointTrajectory()
joint_trajectory.header.stamp = rospy.Time.now()
joint_trajectory.joint_names = list(joint_angle.keys())
joint_trajectory.points = []

# # generate sinusoidal list of data points, two joints moving out of phase
for t in range(0,1):
    trajectory_point = JointTrajectoryPoint()
    trajectory_point.time_from_start = rospy.Duration.from_sec(float(t+0.5))
    trajectory_point.positions = []
    trajectory_point.velocities = []
    trajectory_point.accelerations = []
    trajectory_point.effort = []
    for (key, value) in joint_angle.items():
        joint_position = value * pi / 180
        trajectory_point.positions.append(joint_position)
        trajectory_point.velocities.append(0.0)
        trajectory_point.accelerations.append(0.0)
        trajectory_point.effort.append(0.0)

    joint_trajectory.points.append(trajectory_point)

# Send trajectory to hand_commander
hand_commander.run_joint_trajectory_unsafe(joint_trajectory)

rospy.sleep(rospy.Duration(15))
