#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState


def jackal_pose_publisher():
    rospy.init_node("jackal_pose_pub")

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

        except rospy.ServiceException as e:
            print("Service call failed:" + e)
        pub.publish(pose)
        r.sleep()


if __name__ == "__main__":
    try:
        jackal_pose_publisher()
    except rospy.ROSInterruptException:
        pass
