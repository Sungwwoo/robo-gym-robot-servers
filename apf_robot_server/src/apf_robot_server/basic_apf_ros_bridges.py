#!/usr/bin/env python3

from lidar_based_potential_field.potential_fields import BasicAPF
import grpc
import rospy
import numpy as np
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from concurrent import futures
from robo_gym.utils import apf_env_utils
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2, robot_server_pb2_grpc
from geometry_msgs.msg import Twist, Pose, Pose2D, PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelState, ContactsState
from gazebo_msgs.srv import GetModelState, SetModelState
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import PyKDL
import tf2_ros
import copy
from tf_conversions import posemath
from threading import Event

# State Indecies

# rs_state:
# target = [0.0] * 3
# apf_weights = [0.0] * 2
# robot_pose = [0.0] * 3
# robot_twist = [0.0] * 2
# forces = [0.0] * 3
# collision = False
# obstacles = [0.0] * 21
NUM_OBSTACLES = 3

RS_TARGET = 0
RS_WEIGHTS = RS_TARGET + 3
RS_SCAN = RS_WEIGHTS + 2
RS_ROBOT_POSE = RS_SCAN + 811  # Laser scan length of jackal
RS_ROBOT_TWIST = RS_ROBOT_POSE + 3
RS_FORCES = RS_ROBOT_TWIST + 2
RS_COLLISION = RS_FORCES + 9
RS_OBSTACLES = RS_COLLISION + 1
RS_ROSTIME = RS_OBSTACLES + 3 * NUM_OBSTACLES
RS_PDGAINS = RS_ROSTIME + 1


class RosBridge:
    def __init__(self, real_robot=False):
        ns = rospy.get_namespace()
        self.ns = ns[1 : len(ns) - 1]
        rospy.wait_for_service("/gazebo/get_physics_properties")

        # Event is clear while initialization or set_state is going on
        self.reset = Event()
        self.reset.clear()
        self.get_state_event = Event()
        self.get_state_event.set()
        self.laser_len = 811

        self.real_robot = real_robot

        # Target publisher
        self.pub_target_marker = rospy.Publisher("target_marker", Marker, queue_size=10)
        self.pub_target = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)

        # Obstacle RViz Marker publisher
        self.pub_obstacle_marker = rospy.Publisher("obstacle_marker", MarkerArray, queue_size=10)

        # Rviz Path publisher
        self.pub_base_path = rospy.Publisher("base_exec_path", Path, queue_size=10)

        self.apf_utils = apf_env_utils.APF()

        self.max_lin_vel, self.min_lin_vel = self.apf_utils.get_max_lin_vel(), self.apf_utils.get_min_lin_vel()
        self.max_ang_vel, self.min_ang_vel = self.apf_utils.get_max_ang_vel(), self.apf_utils.get_min_ang_vel()

        self.apf_init_kp, self.apf_init_eta = self.apf_utils.init_kp, self.apf_utils.init_eta
        self.apf_init_linear_kp = self.apf_utils.init_linear_kp
        self.apf_init_angular_kp = self.apf_utils.init_angular_kp
        self.apf_init_angular_kd = self.apf_utils.init_angular_kd

        # Msg for deleting markers
        self.delete_marker = MarkerArray()
        marker = Marker()
        marker.id = 0
        marker.action = Marker.DELETEALL
        self.delete_marker.markers.append(marker)

        # Odometry of the robot subscriber
        if self.real_robot:
            rospy.Subscriber("odom", Odometry, self.cbOdom, queue_size=1)
        else:
            rospy.Subscriber("jackal_velocity_controller/odom", Odometry, self.cbOdom, queue_size=1)  # jackal_velocity_controller/odom

        rospy.Subscriber("base_collision", Bool, self.cbCollision)
        rospy.Subscriber("scan", LaserScan, self.cbScan)
        self.init_odom = rospy.Publisher("set_pose", PoseWithCovarianceStamped, queue_size=10)

        # Reference frame for Path
        self.path_frame = "map"

        if self.real_robot:
            self.path_frame = "world"
            # Apply transform to center the robot, with real_robot we use World frame,
            # World is the Map frame translated in XY to center robot
            tfBuffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tfBuffer)

            trans = tfBuffer.lookup_transform("world", "map", rospy.Time(), rospy.Duration(1.0))
            v = PyKDL.Vector(trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
            r = PyKDL.Rotation.Quaternion(
                trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w
            )
            self.world_to_map = PyKDL.Frame(r, v)

        rospy.Subscriber("robot_pose", Pose, self.callbackState, queue_size=1)

        # Initialize Path
        self.base_path = Path()
        self.base_path.header.stamp = rospy.Time.now()
        self.base_path.header.frame_id = self.path_frame

        # load APF
        self.apf = BasicAPF()

        self.initialize()

        self.rate = rospy.Rate(30)  # 30Hz
        self.apf.run()
        self.reset.set()

    def initialize(self):
        # Robot server states
        self.target = [0.0] * 3
        self.weights = [0.0] * 2
        self.scan = [0.0] * self.laser_len
        self.base_pose = [0.0] * 3
        self.base_twist = [0.0] * 2
        self.forces = [0.0] * 3
        self.collision = False
        self.obstacles = [0.0 for i in range(0, 3 * NUM_OBSTACLES)]
        self.rostime = [0.0]

    def get_state(self):
        """Get states. States are used to calculate reward in step(),

        Returns:
            rs_state:
            target = [0.0] * 3
            apf_weights = [0.0] * 2
            robot_pose = [0.0] * 3
            robot_twist = [0.0] * 2
            forces = [0.0] * 3
            collision = False
            obstacles = [0.0] * 3 * NUM_OBSTACLES
            rostime = [0.0]
        """
        self.get_state_event.clear()

        # Get states
        target = copy.deepcopy(self.target)
        base_pose = copy.deepcopy(self.base_pose)
        base_twist = copy.deepcopy(self.base_twist)
        base_scan = copy.deepcopy(self.scan)
        in_collision = copy.deepcopy(self.collision)
        obstacles = [0.0 for i in range(0, 3 * NUM_OBSTACLES)]
        rostime = [rospy.Time.now().to_sec()]

        # Get forces from apf
        forces = self.apf.get_forces()
        weights = self.apf.get_weights()

        # Clipping Observations (velocity)
        # if base_twist[0] > 0 and base_twist[0] > self.max_lin_vel:
        #     base_twist[0] = self.max_lin_vel
        # elif base_twist[0] < 0 and base_twist[0] < self.min_lin_vel:
        #     base_twist[0] = self.min_lin_vel

        # if base_twist[1] > 0 and base_twist[1] > self.max_ang_vel:
        #     base_twist[1] = self.max_ang_vel
        # elif base_twist[1] < 0 and base_twist[1] < self.min_ang_vel:
        #     base_twist[1] = self.min_ang_vel

        self.get_state_event.set()

        # Create and fill State message
        msg = robot_server_pb2.State()
        msg.state.extend(target)
        msg.state.extend(weights)
        msg.state.extend(base_scan)
        msg.state.extend(base_pose)
        msg.state.extend(base_twist)
        msg.state.extend(forces)
        msg.state.extend([in_collision])
        msg.state.extend(obstacles)
        msg.state.extend(rostime)
        msg.success = 1

        return msg

    def set_state(self, state_msg):
        """Set states using states of reset()
        Args:
            state_msg:
            target = [0.0] * 3
            apf_weights = [0.0] * 2
            robot_pose = [0.0] * 3
            robot_twist = [0.0] * 2
            forces = [0.0] * 3
            collision = False
            obstacles = [0.0] * 3 * NUM_OBSTACLES
        """

        # pause apf
        self.apf.stop()

        # resp = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        # if resp:
        #     rospy.loginfo("Paused gazebo for set_state service")
        # Set environment state
        state = state_msg.state

        # Clear reset Event
        self.reset.clear()

        # Re-initialize Path
        self.base_path = Path()
        self.base_path.header.stamp = rospy.Time.now()
        self.base_path.header.frame_id = self.path_frame

        # Set target internal value
        self.target = copy.deepcopy(state[RS_TARGET : RS_TARGET + 3])

        # Publish Target informations
        self.publish_target(self.target)

        if not self.real_robot:
            # Set Gazebo Robot Model state
            self.set_model_state(self.ns + "_jackal_kinova", copy.deepcopy(state[RS_ROBOT_POSE : RS_ROBOT_POSE + 3]))
            # Set Gazebo Target Model state
            # self.set_model_state("Stop_sign", copy.deepcopy(state[RS_TARGET : RS_TARGET + 3]))
            # Gazebo model repositioning delay
            rospy.sleep(1)
            # Set obstacles poses
            for i in range(0, NUM_OBSTACLES):
                self.set_model_state("unit_cylinder_" + str(i), copy.deepcopy(state[RS_OBSTACLES + 3 * i : RS_OBSTACLES + 3 * (i + 1)]))
        self.publish_obstacle_markers(copy.deepcopy(state[RS_OBSTACLES : RS_OBSTACLES + 3 * NUM_OBSTACLES]))

        # Set Initial weights for apf
        self.apf.set_weights(self.apf_init_kp, self.apf_init_eta)

        # rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.reset_odom(state[RS_ROBOT_POSE + 2])
        # After setting states, enable apf
        self.apf.run()

        # Set reset Event
        self.reset.set()
        return 1

    def publish_obstacle_markers(self, obstacles):
        self.pub_obstacle_marker.publish(self.delete_marker)
        obstacle_markers = MarkerArray()
        for i in range(0, len(obstacles) // 3):
            marker = Marker()
            marker.header.frame_id = "map"
            # marker.header.stamp = rospy.Time.now()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.scale.x = 0.8
            marker.scale.y = 0.8
            marker.scale.z = 0.1
            marker.pose.position.x = obstacles[3 * i]
            marker.pose.position.y = obstacles[3 * i + 1]
            marker.pose.position.z = 0.0
            marker.lifetime = rospy.Duration(0)
            marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = 0, 0, 0, 1
            marker.color.r, marker.color.g, marker.color.b = 0.5, 0.5, 0.5
            marker.color.a = 0.5
            obstacle_markers.markers.append(marker)
        self.pub_obstacle_marker.publish(obstacle_markers)
        return

    def odometry_callback(self, data):
        # Save robot velocities from Odometry internally
        self.robot_twist = data.twist.twist

    def get_robot_state(self):
        # method to get robot position from real base

        return (
            self.robot_pose.x,
            self.robot_pose.y,
            self.robot_pose.theta,
            self.robot_twist.linear.x,
            self.robot_twist.linear.y,
            self.robot_twist.angular.z,
        )

    def set_model_state(self, model_name, state):
        # Set Gazebo Model State
        rospy.wait_for_service("/gazebo/set_model_state")

        start_state = ModelState()
        start_state.model_name = model_name
        start_state.pose.position.x = state[0]
        start_state.pose.position.y = state[1]
        start_state.pose.position.z = 0.0
        orientation = PyKDL.Rotation.RPY(0, 0, state[2])
        (
            start_state.pose.orientation.x,
            start_state.pose.orientation.y,
            start_state.pose.orientation.z,
            start_state.pose.orientation.w,
        ) = orientation.GetQuaternion()

        start_state.twist.linear.x = 0.0
        start_state.twist.linear.y = 0.0
        start_state.twist.linear.z = 0.0
        start_state.twist.angular.x = 0.0
        start_state.twist.angular.y = 0.0
        start_state.twist.angular.z = 0.0

        try:
            set_model_state_client = rospy.ServiceProxy("/gazebo/set_model_state/", SetModelState)
            set_model_state_client(start_state)
        except rospy.ServiceException as e:
            print("Service call failed:" + e)

    def publish_target(self, target_pose):
        # Publish Target RViz Marker
        t_marker = Marker()
        t_marker.type = 2  # =>SPHERE
        t_marker.scale.x = 0.3
        t_marker.scale.y = 0.3
        t_marker.scale.z = 0.3
        t_marker.action = 0
        t_marker.frame_locked = 1
        t_marker.pose.position.x = target_pose[0]
        t_marker.pose.position.y = target_pose[1]
        t_marker.pose.position.z = 0.0
        rpy_orientation = PyKDL.Rotation.RPY(0.0, 0.0, target_pose[2])
        q_orientation = rpy_orientation.GetQuaternion()
        t_marker.pose.orientation.x = q_orientation[0]
        t_marker.pose.orientation.y = q_orientation[1]
        t_marker.pose.orientation.z = q_orientation[2]
        t_marker.pose.orientation.w = q_orientation[3]
        t_marker.id = 0
        t_marker.header.stamp = rospy.Time.now()
        t_marker.header.frame_id = self.path_frame
        t_marker.color.a = 1.0
        t_marker.color.r = 0.0  # red
        t_marker.color.g = 1.0
        t_marker.color.b = 0.0

        target = PoseStamped()
        target.header.frame_id = self.path_frame
        target.header.stamp = rospy.Time.now()
        target.pose.position = Point(target_pose[0], target_pose[1], 0)
        target.pose.orientation.x = q_orientation[0]
        target.pose.orientation.y = q_orientation[1]
        target.pose.orientation.z = q_orientation[2]
        target.pose.orientation.w = q_orientation[3]

        self.pub_target_marker.publish(t_marker)
        self.pub_target.publish(target)

    def reset_odom(self, yaw):
        zeropoint = PoseWithCovarianceStamped()

        zeropoint.header.frame_id = self.ns + "/odom"

        zeropoint.pose.pose.position.x = 0.0
        zeropoint.pose.pose.position.y = 0.0
        zeropoint.pose.pose.position.z = 0.0

        zeropoint.pose.pose.orientation.x = 0.0
        zeropoint.pose.pose.orientation.y = 0.0
        zeropoint.pose.pose.orientation.z = 0.0
        zeropoint.pose.pose.orientation.w = 0.0

        orientation = PyKDL.Rotation.RPY(0, 0, yaw)
        (
            zeropoint.pose.pose.orientation.x,
            zeropoint.pose.pose.orientation.y,
            zeropoint.pose.pose.orientation.z,
            zeropoint.pose.pose.orientation.w,
        ) = orientation.GetQuaternion()

        zeropoint.pose.covariance[0:35] = [0.0] * 35
        rate = rospy.Rate(2)
        count = 0

        while not rospy.is_shutdown():
            self.init_odom.publish(zeropoint)
            rate.sleep()
            count = count + 1
            if count == 3:
                count = 0
                break

    def callbackState(self, data):
        # If state is not being reset proceed otherwise skip callback
        if self.reset.isSet():
            if self.real_robot:
                # Convert Pose from relative to Map to relative to World frame
                f_r_in_map = posemath.fromMsg(data)
                f_r_in_world = self.world_to_map * f_r_in_map
                data = posemath.toMsg(f_r_in_world)

            x = data.position.x
            y = data.position.y

            orientation = PyKDL.Rotation.Quaternion(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)

            euler_orientation = orientation.GetRPY()
            yaw = euler_orientation[2]

            # Append Pose to Path
            stamped_base_pose = PoseStamped()
            stamped_base_pose.pose = data
            stamped_base_pose.header.stamp = rospy.Time.now()
            stamped_base_pose.header.frame_id = self.path_frame
            self.base_path.poses.append(stamped_base_pose)
            self.pub_base_path.publish(self.base_path)

            # Update internal Pose variable
            self.base_pose = copy.deepcopy([x, y, yaw])
        else:
            pass

    def cbOdom(self, data):
        lin_vel = data.twist.twist.linear.x
        ang_vel = data.twist.twist.angular.z

        # Update internal Twist variable
        self.base_twist = copy.deepcopy([lin_vel, ang_vel])

    def cbCollision(self, data):
        if data.data == False:
            self.collision = False
        else:
            self.collision = True

    def cbScan(self, scan):
        if self.get_state_event.isSet():
            ranges = scan.ranges
            self.scan = copy.deepcopy(ranges)
        else:
            pass

    def set_params(self, KP, ETA):
        rospy.sleep(0.05)
        self.apf.set_weights(KP, ETA)


class RosBridge_with_PD(RosBridge):
    def initialize(self):
        # Robot server states
        self.target = [0.0] * 3
        self.weights = [0.0] * 2
        self.base_scan = [0.0] * self.laser_len
        self.base_pose = [0.0] * 3
        self.base_twist = [0.0] * 2
        self.forces = [0.0] * 3
        self.collision = False
        self.obstacles = [0.0 for i in range(0, 3 * NUM_OBSTACLES)]
        self.rostime = [0.0]
        self.pd_gains = [0.0] * 3

    def get_state(self):
        """Get states. States are used to calculate reward in step(),

        Returns:
            rs_state:
            target = [0.0] * 3
            apf_weights = [0.0] * 2
            robot_pose = [0.0] * 3
            robot_twist = [0.0] * 2
            forces = [0.0] * 3
            collision = False
            obstacles = [0.0] * 3 * NUM_OBSTACLES
            rostime = [0.0]
            pd_gains = [0.0] * 3
        """
        self.get_state_event.clear()

        # Get states
        target = copy.deepcopy(self.target)
        base_scan = copy.deepcopy(self.scan)
        base_pose = copy.deepcopy(self.base_pose)
        base_twist = copy.deepcopy(self.base_twist)
        in_collision = copy.deepcopy(self.collision)
        obstacles = [0.0 for i in range(0, 3 * NUM_OBSTACLES)]
        rostime = [rospy.Time.now().to_sec()]

        # Get forces from apf
        forces = self.apf.get_forces()
        weights = self.apf.get_weights()
        pd_gains = self.apf.get_gains()

        # Clipping Observations (velocity)
        if base_twist[0] > 0 and base_twist[0] > self.max_lin_vel:
            base_twist[0] = self.max_lin_vel
        elif base_twist[0] < 0 and base_twist[0] < self.min_lin_vel:
            base_twist[0] = self.min_lin_vel

        if base_twist[1] > 0 and base_twist[1] > self.max_ang_vel:
            base_twist[1] = self.max_ang_vel
        elif base_twist[1] < 0 and base_twist[1] < self.min_ang_vel:
            base_twist[1] = self.min_ang_vel

        self.get_state_event.set()

        # Create and fill State message
        msg = robot_server_pb2.State()
        msg.state.extend(target)
        msg.state.extend(weights)
        msg.state.extend(base_scan)
        msg.state.extend(base_pose)
        msg.state.extend(base_twist)
        msg.state.extend(forces)
        msg.state.extend([in_collision])
        msg.state.extend(obstacles)
        msg.state.extend(rostime)
        msg.state.extend(pd_gains)
        msg.success = 1

        return msg

    def set_state(self, state_msg):
        """Set states using states of reset()
        Args:
            state_msg:
            target = [0.0] * 3
            apf_weights = [0.0] * 2
            robot_pose = [0.0] * 3
            robot_twist = [0.0] * 2
            forces = [0.0] * 3
            collision = False
            obstacles = [0.0] * 3 * NUM_OBSTACLES
            rostime = [0.0]
            pd_gains = [0.0] * 3
        """

        # pause apf
        self.apf.stop()

        resp = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        if resp:
            rospy.loginfo("Paused gazebo for set_state service")
        # Set environment state
        state = state_msg.state

        # Clear reset Event
        self.reset.clear()

        # Re-initialize Path
        self.base_path = Path()
        self.base_path.header.stamp = rospy.Time.now()
        self.base_path.header.frame_id = self.path_frame

        # Set target internal value
        self.target = copy.deepcopy(state[RS_TARGET : RS_TARGET + 3])

        # Publish Target informations
        self.publish_target(self.target)

        if not self.real_robot:
            # Set Gazebo Robot Model state
            self.set_model_state(self.ns + "_jackal_kinova", copy.deepcopy(state[RS_ROBOT_POSE : RS_ROBOT_POSE + 3]))
            # Set Gazebo Target Model state
            # self.set_model_state("Stop_sign", copy.deepcopy(state[RS_TARGET : RS_TARGET + 3]))
            # Gazebo model repositioning delay
            rospy.sleep(1)
            # Set obstacles poses
            for i in range(0, NUM_OBSTACLES):
                self.set_model_state("unit_cylinder_" + str(i), copy.deepcopy(state[RS_OBSTACLES + 3 * i : RS_OBSTACLES + 3 * (i + 1)]))
        self.publish_obstacle_markers(copy.deepcopy(state[RS_OBSTACLES : RS_OBSTACLES + 3 * NUM_OBSTACLES]))

        # Set Initial weights for apf
        self.apf.set_weights(self.apf_init_kp, self.apf_init_eta)
        self.apf.set_gains(self.apf_init_linear_kp, self.apf_init_angular_kp, self.apf_init_angular_kd)

        rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.reset_odom(state[RS_ROBOT_POSE + 2])
        # After setting states, enable apf
        self.apf.run()

        # Set reset Event
        self.reset.set()
        return 1

    def set_params(self, KP, ETA, linear_kp, angular_kp, angular_kd):
        rospy.sleep(0.05)
        self.apf.set_weights(KP, ETA)
        self.apf.set_gains(linear_kp, angular_kp, angular_kd)
