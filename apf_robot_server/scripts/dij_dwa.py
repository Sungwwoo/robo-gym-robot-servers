#!/usr/bin/env python3

import numpy as np
import rospy
from gazebo_msgs.msg import ModelState, ContactsState
from gazebo_msgs.srv import GetModelState, SetModelState

start_points = [
    [1.5, 10.0, 0.0],
    [1.5, 4.0, 0.0],
    [1.5, -4.0, 0.0],
    [1.5, -10.0, 0.0],
    [-9.5, 10.0, 0.0],
    [-9.5, 4.0, np.pi / 2],
    [-9.5, -4.0, 0.0],
    [-9.5, -10.0, 0.0],
]

target_points = [
    [8.75, 10.0, 0.0],
    [8.75, 4.0, 0.0],
    [8.75, -4.0, 0.0],
    [8.75, -10.0, 0.0],
    [-2.75, 10.0, 0.0],
    [-2.75, 3.0, 0.0],
    [-2.75, -4.0, 0.0],
    [-2.75, -10.0, 0.0],
]
