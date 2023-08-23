#!/usr/bin/env python3

import grpc
import rospy
from concurrent import futures
from robo_gym.utils import jackal_kinova_utils
from robo_gym_server_modules.robot_server.grpc_msgs.python import robot_server_pb2, robot_server_pb2_grpc
from geometry_msgs.msg import Twist, Pose, Pose2D, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState, ContactsState
from gazebo_msgs.srv import GetModelState, SetModelState
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
import PyKDL
import tf2_ros
import copy
from tf_conversions import posemath
from threading import Event
import numpy as np

# self.target = [0.0] * 3
# self.base_pose = [0.0] * 3
# self.base_twist = [0.0] * 2
# self.scan = [0.0] * self.laser_len
# self.collision = False
# self.obstacle_0 = [0.0] * 3
# self.obstacle_1 = [0.0] * 3
# self.obstacle_2 = [0.0] * 3
# self.obstacle_3 = [0.0] * 3
# self.obstacle_4 = [0.0] * 3
# self.obstacle_5 = [0.0] * 3
# self.obstacle_6 = [0.0] * 3

NUM_OBSTACLES = 3
DIST_BTW_OBSTACLES = 2.3

RS_TARGET = 0
RS_ROBOT_POSE = 3
RS_ROBOT_TWIST = 6
RS_SCAN = 8
RS_COLLISION = 8 + 811  # Raser Scan Length
RS_OBSTACLES = RS_COLLISION + 1
RS_ROSTIME = RS_OBSTACLES + 3 * NUM_OBSTACLES


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
            lin_vel, ang_vel = self.rosbridge.publish_env_cmd_vel(request.action[0], request.action[1])
            return robot_server_pb2.Success(success=1)
        except:
            return robot_server_pb2.Success(success=0)


def serve():
    server_port = rospy.get_param("~server_port")
    real_robot = rospy.get_param("~real_robot")
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    robot_server_pb2_grpc.add_RobotServerServicer_to_server(RobotServerServicer(real_robot=real_robot), server)
    server.add_insecure_port("[::]:" + repr(server_port))
    server.start()
    if real_robot:
        rospy.loginfo("Real Robot Server started at " + repr(server_port))
    else:
        rospy.loginfo("Sim Robot Server started at " + repr(server_port))
    rospy.spin()


class RosBridge:
    def __init__(self, real_robot=False):
        ns = rospy.get_namespace()
        self.ns = ns[1 : len(ns) - 1]
        # Event is clear while initialization or set_state is going on
        self.reset = Event()
        self.reset.clear()
        self.get_state_event = Event()
        self.get_state_event.set()
        self.laser_len = 811

        self.real_robot = real_robot
        # cmd_vel_command_handler publisher
        self.env_cmd_vel_pub = rospy.Publisher("env_cmd_vel", Twist, queue_size=1)
        # Target RViz Marker publisher
        self.target_pub = rospy.Publisher("target_marker", Marker, queue_size=10)
        # Obstacle RViz Marker publisher
        self.pub_obstacle_marker = rospy.Publisher("obstacle_marker", MarkerArray, queue_size=10)

        # Rviz Path publisher
        self.base_exec_path = rospy.Publisher("base_exec_path", Path, queue_size=10)

        self.init_odom = rospy.Publisher("set_pose", PoseWithCovarianceStamped, queue_size=10)

        self.jackal_kinova = jackal_kinova_utils.Jackal_Kinova()
        self.max_lin_vel, self.min_lin_vel = self.jackal_kinova.get_max_lin_vel(), self.jackal_kinova.get_min_lin_vel()
        self.max_ang_vel, self.min_ang_vel = self.jackal_kinova.get_max_ang_vel(), self.jackal_kinova.get_min_ang_vel()

        # Msg for deleting markers
        self.delete_marker = MarkerArray()
        marker = Marker()
        marker.id = 0
        marker.action = Marker.DELETEALL
        self.delete_marker.markers.append(marker)

        # Odometry of the robot subscriber
        if self.real_robot:
            rospy.Subscriber("odom", Odometry, self.callbackOdometry, queue_size=1)
        else:
            rospy.Subscriber("jackal_velocity_controller/odom", Odometry, self.callbackOdometry, queue_size=1)  # jackal_velocity_controller/odom

        rospy.Subscriber("scan", LaserScan, self.LaserScan_callback)
        rospy.Subscriber("base_collision", Bool, self.collision_callback)

        self.target = [0.0] * 3
        self.base_pose = [0.0] * 3
        self.base_twist = [0.0] * 2
        self.scan = [0.0] * self.laser_len
        self.collision = False
        self.obstacles = [0.0 for i in range(0, 3 * NUM_OBSTACLES)]
        self.rostime = [0.0]

        self.current_obstacles = [0.0 for i in range(0, 3 * NUM_OBSTACLES)]

        self.state_length = len(
            self.target + self.base_pose + self.base_twist + self.scan + [self.collision] + self.obstacles + self.rostime,
        )
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

        # Flag indicating if it is safe to move backwards
        # self.safe_to_move_back = True
        # Flag indicating if it is safe to move forward
        self.safe_to_move_front = True
        self.rate = rospy.Rate(30)  # 30Hz
        self.reset.set()

    def get_state(self):
        self.get_state_event.clear()
        # Get environment state
        state = [0.0 for i in range(0, self.state_length)]

        state[RS_TARGET : RS_TARGET + 3] = copy.deepcopy(self.target)
        state[RS_ROBOT_POSE : RS_ROBOT_POSE + 3] = copy.deepcopy(self.base_pose)
        state[RS_ROBOT_TWIST : RS_ROBOT_TWIST + 2] = copy.deepcopy(self.base_twist)
        state[RS_SCAN : RS_SCAN + self.laser_len] = copy.deepcopy(self.scan)
        state[RS_COLLISION] = [copy.deepcopy(self.collision)]
        state[RS_OBSTACLES : RS_OBSTACLES + 3 * NUM_OBSTACLES] = self.current_obstacles
        state[RS_ROSTIME] = [rospy.Time.now().to_sec()]

        target = copy.deepcopy(self.target)
        base_pose = copy.deepcopy(self.base_pose)
        base_twist = copy.deepcopy(self.base_twist)
        base_scan = copy.deepcopy(self.scan)
        in_collision = [copy.deepcopy(self.collision)]
        obstacles = self.current_obstacles
        rostime = [rospy.Time.now().to_sec()]

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
        # msg.state = state[:]
        msg.state.extend(target)
        msg.state.extend(base_pose)
        msg.state.extend(base_twist)
        msg.state.extend(base_scan)
        msg.state.extend(in_collision)
        msg.state.extend(obstacles)
        msg.state.extend(rostime)
        msg.success = 1

        return msg

    def set_state(self, state_msg):
        # Set environment state
        state = state_msg.state
        resp = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        if resp:
            rospy.loginfo("Paused gazebo for set_state service")
        # Clear reset Event
        self.reset.clear()
        # Re-initialize Path
        self.base_path = Path()
        self.base_path.header.stamp = rospy.Time.now()
        self.base_path.header.frame_id = self.path_frame

        # Set target internal value
        self.target = copy.deepcopy(state[RS_TARGET : RS_TARGET + 3])
        # Publish Target Marker
        self.publish_target_marker(self.target)

        if not self.real_robot:
            # Set Gazebo Robot Model state
            self.set_model_state(self.ns + "_jackal_kinova", copy.deepcopy(state[RS_ROBOT_POSE : RS_ROBOT_POSE + 3]))
            rospy.sleep(1)

            # Set Gazebo Target Model state
            # self.set_model_state("Stop_sign", copy.deepcopy(state[0:3]))
            # Set obstacles poses
            rospy.loginfo("Setting obstacle location")
            self.current_obstacles = state[RS_OBSTACLES : RS_OBSTACLES + 3 * NUM_OBSTACLES]
            for i in range(0, NUM_OBSTACLES):
                self.set_model_state("unit_cylinder_" + str(i), copy.deepcopy(state[RS_OBSTACLES + 3 * i : RS_OBSTACLES + 3 * (i + 1)]))
            self.publish_obstacle_markers(copy.deepcopy(state[RS_OBSTACLES : RS_OBSTACLES + 3 * NUM_OBSTACLES]))

        rospy.loginfo("obstacle location set")

        rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.reset_odom(0.0)

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

    def publish_env_cmd_vel(self, lin_vel, ang_vel):
        if not self.safe_to_move_front:
            # If it is not safe to move overwrite velocities and stop robot
            rospy.sleep(0.05)
            return 0.0, 0.0
        msg = Twist()
        msg.linear.x = lin_vel
        msg.angular.z = ang_vel
        self.env_cmd_vel_pub.publish(msg)
        # rospy.loginfo("Received Action: " + str(lin_vel) + "\t" + str(ang_vel) + "\n")
        # Sleep time set manually to achieve approximately 10Hz rate

        rospy.sleep(0.05)
        return lin_vel, ang_vel

    def odometry_callback(self, data):
        # Save robot velocities from Odometry internally
        self.robot_twist = data.twist.twist

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

    def publish_target_marker(self, target_pose):
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
        self.target_pub.publish(t_marker)

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
            self.base_exec_path.publish(self.base_path)

            # Update internal Pose variable
            self.base_pose = copy.deepcopy([x, y, yaw])
        else:
            pass

    def callbackOdometry(self, data):
        lin_vel = data.twist.twist.linear.x
        ang_vel = data.twist.twist.angular.z

        # Update internal Twist variable
        self.base_twist = copy.deepcopy([lin_vel, ang_vel])
        # rospy.loginfo("Current State: " + str(lin_vel) + "\t" + str(ang_vel) + "\n")

    def LaserScan_callback(self, data):
        if self.get_state_event.isSet():
            scan = data.ranges
            # scan = scan[8:len(scan)-8] # when you want remove first and last scan data activate this line
            # =list(filter(lambda a: a != 0.0, scan))   # remove all 0.0 values that are at beginning and end of scan list
            scan = np.array(scan)
            scan = np.nan_to_num(scan)
            scan = np.clip(scan, data.range_min, data.range_max)
            self.scan = copy.deepcopy(scan.tolist())
            self.safe_to_move_front = all(i >= 0.04 for i in scan)
        else:
            pass

    def collision_callback(self, data):
        if data.data == False:
            self.collision = False
        else:
            self.collision = True


if __name__ == "__main__":
    try:
        rospy.init_node("robot_server")
        rospy.loginfo("Waiting 10s before starting initialization of robot_server")
        rospy.sleep(10)
        rospy.loginfo("Initializing robot_server node")
        serve()
    except (KeyboardInterrupt, SystemExit):
        pass
