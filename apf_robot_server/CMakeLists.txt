cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(apf_robot_server)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_python_setup()
catkin_install_python(PROGRAMS
  scripts/robot_pose_publisher.py
  scripts/pub_goal_tf.py
  scripts/robot_server.py
  scripts/robot_server_with_pd.py
  scripts/clpf_robot_server.py
  scripts/clpf_robot_server_with_pd.py
  scripts/contact_handler.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})