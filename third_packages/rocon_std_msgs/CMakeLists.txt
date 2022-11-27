##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_std_msgs)

##############################################################################
# Dependencies
##############################################################################

find_package(catkin REQUIRED COMPONENTS 
    message_generation
    std_msgs
)

##############################################################################
# Messgaes
##############################################################################

add_message_files(
  DIRECTORY msg
  FILES
  StringArray.msg
  Strings.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  EmptyString.srv
)

generate_messages(DEPENDENCIES std_msgs)

##############################################################################
# Package
##############################################################################

catkin_package(CATKIN_DEPENDS
    message_runtime
    std_msgs
)
