#!/usr/bin/env python3

import grpc
import rospy
from concurrent import futures
from robo_gym.utils import jackal_kinova_utils
from robo_gym_server_modules.robot_server.grpc_msgs.python import (
    robot_server_pb2,
    robot_server_pb2_grpc,
)
from geometry_msgs.msg import (
    Twist,
    Pose,
    Pose2D,
    PoseStamped,
    PoseWithCovarianceStamped,
)
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState, ContactsState
from gazebo_msgs.srv import GetModelState, SetModelState
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from jackal_kinova_robot_server.ros_bridge_no_obstacle import RosBridge
import PyKDL
import tf2_ros
import copy
from tf_conversions import posemath
from threading import Event
import numpy as np


class RobotServerServicer(robot_server_pb2_grpc.RobotServerServicer):
    def __init__(self, real_robot):
        self.rosbridge = RosBridge(real_robot=real_robot)

    def GetState(self, request, context):
        try:
            return self.rosbridge.get_state()
        except:
            return robot_server_pb2.State(success=0)

    def SetState(self, request, context):
        try:
            s = self.rosbridge.set_state(state_msg=request)
            return robot_server_pb2.Success(success=1)
        except:
            return robot_server_pb2.Success(success=0)

    def SendAction(self, request, context):
        try:
            lin_vel, ang_vel = self.rosbridge.publish_env_cmd_vel(
                request.action[0], request.action[1]
            )
            return robot_server_pb2.Success(success=1)
        except:
            return robot_server_pb2.Success(success=0)


def serve():
    server_port = rospy.get_param("~server_port")
    real_robot = rospy.get_param("~real_robot")
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    robot_server_pb2_grpc.add_RobotServerServicer_to_server(
        RobotServerServicer(real_robot=real_robot), server
    )
    server.add_insecure_port("[::]:" + repr(server_port))
    server.start()
    if real_robot:
        rospy.loginfo("Real Robot Server started at " + repr(server_port))
    else:
        rospy.loginfo("Sim Robot Server started at " + repr(server_port))
    rospy.spin()


if __name__ == "__main__":
    try:
        rospy.init_node("robot_server")
        rospy.loginfo("Waiting 10s before starting initialization of robot_server")
        rospy.sleep(10)
        rospy.loginfo("Initializing robot_server node")
        serve()
    except (KeyboardInterrupt, SystemExit):
        pass
