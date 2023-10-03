#!/usr/bin/env python3

import numpy as np
import rospy
import PyKDL
import copy
from lidar_based_potential_field.ros_utils import calcDistance
from gazebo_msgs.msg import ModelState, ContactsState
from gazebo_msgs.srv import GetModelState, SetModelState, DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty

from lidar_based_potential_field.potential_fields import ClusteredAPF


zeropoint = PoseWithCovarianceStamped()

zeropoint.header.frame_id = "map"

zeropoint.pose.pose.position.x = 0.0
zeropoint.pose.pose.position.y = 0.0
zeropoint.pose.pose.position.z = 0.0

zeropoint.pose.pose.orientation.x = 0.0
zeropoint.pose.pose.orientation.y = 0.0
zeropoint.pose.pose.orientation.z = 0.0
zeropoint.pose.pose.orientation.w = 0.0

orientation = PyKDL.Rotation.RPY(0, 0, 0)
(
    zeropoint.pose.pose.orientation.x,
    zeropoint.pose.pose.orientation.y,
    zeropoint.pose.pose.orientation.z,
    zeropoint.pose.pose.orientation.w,
) = orientation.GetQuaternion()

zeropoint.pose.covariance[0:36] = [0.0 for i in range(36)]

start_points = [
    [1.5, 10.0, 0.0],
    [1.5, 4.0, 0.0],
    [1.5, -4.0, 0.0],
    [1.5, -10.0, 0.0],
    [-9.5, 10.0, 0.0],
    [-9.25, 4.0, 0],
    [-9.25, -4.0, 0.0],
    [-9.25, -10.0, 0.0],
]

target_points = [
    [8.75, 10.0, 0.0],
    [8.75, 4.0, 0.0],
    [8.75, -4.0, 0.0],
    [8.75, -10.0, 0.0],
    [-2.75, 10.0, 0.0],
    [-2.5, 3.0, 0.0],
    [-2.75, -4.0, 0.0],
    [-2.5, -10.0, 0.0],
]

in_collision = False
base_pose = [0.0 for i in range(0, 3)]


def cbCollision(collision):
    global in_collision
    in_collision = collision.data


def reset_odom():
    rate = rospy.Rate(2)
    count = 0
    while not rospy.is_shutdown():
        init_odom.publish(zeropoint)
        rate.sleep()
        count = count + 1
        if count == 3:
            break


def cbPose(self, data):
    global base_pose

    x = data.position.x
    y = data.position.y

    orientation = PyKDL.Rotation.Quaternion(
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w,
    )

    euler_orientation = orientation.GetRPY()
    yaw = euler_orientation[2]

    # Update internal Pose variable
    base_pose = copy.deepcopy([x, y, yaw])


def publish_target(target_pose):
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
    t_marker.header.frame_id = "map"
    t_marker.color.a = 1.0
    t_marker.color.r = 0.0
    t_marker.color.g = 1.0
    t_marker.color.b = 0.0

    target = PoseStamped()
    target.header.frame_id = "map"
    target.header.stamp = rospy.Time.now()
    target.pose.position = Point(target_pose[0], target_pose[1], 0)
    target.pose.orientation.x = q_orientation[0]
    target.pose.orientation.y = q_orientation[1]
    target.pose.orientation.z = q_orientation[2]
    target.pose.orientation.w = q_orientation[3]
    rospy.loginfo(
        "Published target {:.3f}, {:.3f}, {:.3f}".format(
            target_pose[0],
            target_pose[1],
            target_pose[2],
        )
    )
    for i in range(2):
        pub_target_marker.publish(t_marker)
        pub_target.publish(target)
        rospy.sleep(0.5)


def cbOdom(data):
    lin_vel = data.twist.twist.linear.x
    ang_vel = data.twist.twist.angular.z

    # Update Twist variable
    base_twist = copy.deepcopy([lin_vel, ang_vel])


runs = 15
if __name__ == "__main__":
    rospy.init_node("experiment_move_base")
    apf = ClusteredAPF()

    # publishers
    pub_target = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
    pub_target_marker = rospy.Publisher("target_marker", Marker, queue_size=10)
    init_odom = rospy.Publisher("set_pose", PoseWithCovarianceStamped, queue_size=10)
    cancle_goal = rospy.Publisher("move_base/cancel", GoalID, queue_size=10)
    init_pose = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=10)

    # subscribers
    rospy.Subscriber("base_collision", Bool, cbCollision)

    room = 4

    success = 0
    sum_time = 0.0
    sum_path_length = 0.0
    sum_total_acc_lin = 0.0
    sum_total_acc_ang = 0.0
    best_path_length = 1000
    best_time = 1000
    f_summary = open("logs_clpf/room" + str(room) + "/experiment_summary.txt", "w")

    for iteration in range(0, runs):
        input("Press Enter to continue")
        episode_time = 0.0
        path_length = 0.0
        total_acc_lin = 0.0
        total_acc_ang = 0.0
        prev_v = 0.0
        prev_w = 0.0
        prev_base_pose = [0.0 for i in range(0, 3)]
        failed = False
        base_pose = [0.0 for i in range(0, 3)]

        # Open a new log file
        fileName = "logs_clpf/room" + str(room) + "/experiment_run_" + str(iteration + 1) + ".txt"
        f_log = open(fileName, "w")

        # Set model at starting point
        rospy.wait_for_service("/gazebo/set_model_state")

        start_state = ModelState()
        start_state.model_name = "jackal_kinova"
        start_state.pose.position.x = start_points[room][0]
        start_state.pose.position.y = start_points[room][1]
        start_state.pose.position.z = 0.0
        orientation = PyKDL.Rotation.RPY(0, 0, start_points[room][2])
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

        rospy.sleep(np.random.randint(2, 6))
        rospy.sleep(3)

        in_collision = False
        # Publish target to move_base
        publish_target([a - b for a, b in zip(target_points[room], start_points[room])])

        apf.run()
        # Set ROSTime
        startTime = rospy.Time.now().to_sec()
        prevTime = startTime

        # Use target_points for apfs, otherwise relative_target
        while calcDistance(target_points[room][0:2], base_pose[0:2]) > 0.3:
            rospy.loginfo(
                "Current Pose: {:.3f}, {:.3f}, {:.3f}".format(
                    base_pose[0], base_pose[1], base_pose[2]
                )
            )
            if in_collision:
                rospy.loginfo("Collision!")
                failed = True
                f_log.write("Failed!")
                f_log.close()
                break

            if episode_time > 60.0:
                rospy.loginfo("Max Time Triggered!")
                failed = True
                f_log.write("Failed!")
                f_log.close()
                break
            # log rostime, base_pose, velocity, acceleration
            data = rospy.wait_for_message("robot_pose", Pose)
            x = data.position.x
            y = data.position.y

            orientation = PyKDL.Rotation.Quaternion(
                data.orientation.x,
                data.orientation.y,
                data.orientation.z,
                data.orientation.w,
            )

            euler_orientation = orientation.GetRPY()
            yaw = euler_orientation[2]
            base_pose = copy.deepcopy([x, y, yaw])
            currentTime = rospy.Time.now().to_sec()
            pose_diff = [a - b for a, b in zip(base_pose, prev_base_pose)]

            rospy.loginfo(
                "Pose Difference to target: {:.3f}, {:.3f}, {:.3f}".format(
                    pose_diff[0], pose_diff[1], pose_diff[2]
                )
            )
            timeDiff = currentTime - prevTime
            episode_time += timeDiff

            rospy.loginfo(
                "Distance left: %f" % calcDistance(target_points[room][0:2], base_pose[0:2])
            )

            ds = np.hypot(pose_diff[0], pose_diff[1])
            v = ds / timeDiff
            acc_lin = (v - prev_v) / timeDiff
            w = pose_diff[2] / timeDiff
            acc_ang = (w - prev_w) / timeDiff

            f_log.write(
                "{:.3f}\t{:.3f}\t{:.3f}\t{:.3f}\t{:.3f}\t{:.3f}\t{:.3f}\n".format(
                    timeDiff, base_pose[0], base_pose[1], v, w, acc_lin, acc_ang
                )
            )
            total_acc_lin += abs(acc_lin)
            total_acc_ang += abs(acc_ang)
            path_length += ds
            prev_v = v
            prev_w = w
            prevTime = currentTime
            prev_base_pose = base_pose

        apf.stop()

        if not failed:
            success += 1
            endTime = rospy.Time.now().to_sec()
            total_time = endTime - startTime
            sum_time += total_time
            sum_path_length += path_length
            sum_total_acc_ang += total_acc_ang
            sum_total_acc_lin += total_acc_lin

            if best_path_length > path_length:
                best_path_length = path_length
            if best_time > total_time:
                best_time = total_time

            f_log.write("Succeeded!\n")
            f_log.write("Path Length:" + str(path_length))
            f_log.write("\nElapsed Time:" + str(total_time))

            f_log.write("\nTotal_acc: {:.3f}, {:.3f}".format(total_acc_lin, total_acc_ang))
            f_summary.write("{:.3f}, {:.3f}\n".format(path_length, total_time))
            f_log.close()

    f_summary.write("Success Rate: {}%\n".format(success / runs * 100))
    f_summary.write("Average Path Length: {:.3f} m\n".format((sum_path_length / success)))
    f_summary.write("Best Path Length: {:.3f} m\n".format(best_path_length))
    f_summary.write("Average Time: {:.3f} sec\n".format((sum_time / success)))
    f_summary.write("Best Time: {:.3f} sec".format(best_time))
    f_summary.write(
        "\nTotal Accleration: {:.3f}, {:.3f}".format(
            sum_total_acc_lin / success, sum_total_acc_ang / success
        )
    )
    f_summary.close()
