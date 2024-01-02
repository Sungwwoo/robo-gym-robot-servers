#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState
import tf2_ros
import lidar_based_potential_field.ros_utils as ros_utils


def jackal_pose_publisher():
    ns = rospy.get_namespace()
    if ns == "/":
        robot_name = "jackal_kinova"
    else:
        ns = ns[1 : len(ns) - 1]
        robot_name = ns + "_jackal_kinova"
    pub = rospy.Publisher("robot_pose", Pose, queue_size=10)
    r = rospy.Rate(10.0)
    rospy.wait_for_service("/gazebo/get_model_state")
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    while not rospy.is_shutdown():
        try:
            model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            model_coordinates = model_state(robot_name, "")
            pose = Pose()
            pose.position.x = model_coordinates.pose.position.x
            pose.position.y = model_coordinates.pose.position.y
            pose.position.z = model_coordinates.pose.position.z
            pose.orientation.x = model_coordinates.pose.orientation.x
            pose.orientation.y = model_coordinates.pose.orientation.y
            pose.orientation.z = model_coordinates.pose.orientation.z
            pose.orientation.w = model_coordinates.pose.orientation.w
            # loc = GetRobotPose()
            # pose.position.x = loc[0][0]
            # pose.position.y = loc[0][1]
            # pose.position.z = 0.0
            # pose.orientation.x = loc[1][0]
            # pose.orientation.y = loc[1][1]
            # pose.orientation.z = loc[1][2]
            # pose.orientation.w = loc[1][3]
        except rospy.ServiceException as e:
            print("Service call failed:" + e)
        pub.publish(pose)
        r.sleep()


def GetRobotPose():
    """Return the current location of robot.

    Return:
        [Location_x, Location_y],
        [Orientation_x, Orientation_y, Orientation_z, Orientation_z]
        based on odom frame
    """
    loc = ros_utils.GetTF(tfBuffer, "odom", "base_link")
    robotLocation = [loc.transform.translation.x, loc.transform.translation.y]
    robotOrientation = [
        loc.transform.rotation.x,
        loc.transform.rotation.y,
        loc.transform.rotation.z,
        loc.transform.rotation.w,
    ]
    return robotLocation, robotOrientation


if __name__ == "__main__":
    rospy.init_node("jackal_pose_pub")
    tfBuffer = tf2_ros.Buffer()
    tfListner = tf2_ros.TransformListener(tfBuffer)

    try:
        jackal_pose_publisher()
    except rospy.ROSInterruptException:
        pass
