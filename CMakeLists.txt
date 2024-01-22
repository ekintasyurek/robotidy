cmake_minimum_required(VERSION 3.0.2)
project(robotidy)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  rospy
  std_msgs
  message_generation
  actionlib_msgs
  trajectory_msgs
)

# Uncomment this if the package has a setup.py
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  kpkv_msg.srv
)

add_action_files(
  FILES
  traj.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  trajectory_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS actionlib rospy std_msgs message_runtime actionlib_msgs trajectory_msgs
)

###########
## Build ##
###########

# No need to specify include directories for Python

# No need to declare C++ executables or libraries

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  src/controller.py
  src/kpkv_service_client.py
  src/trajectory_action_server.py
  src/trajectory_action_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Uncomment and update to install other project files like launch files
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

# Add folders to be run by python nosetests
# catkin_add_nosetests(test)
