#cmake version
cmake_minimum_required(VERSION 3.2)

#project name
set(ROCINANTE_NAME rocinante)
project(${ROCINANTE_NAME})

# As moc files are generated in the binary dir, tell CMake
# to always look for includes there:
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# compatibility with c++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# // ------------------------------------------------------------------------------------ //
# // --------------------------------------- ROS ---------------------------------------- //
# // ------------------------------------------------------------------------------------ //
if(DEFINED CATKIN_DEVEL_PREFIX)
#  #––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––#
#  #      adding all the ROS elements inside this IF conditional      #
#  #––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––#
#  ## Find catkin macros and libraries
#  ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
#  ## is used, also find other catkin packages
  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    genmsg
    message_generation
    nav_msgs
    roscpp
    std_msgs
    velodyne_laserscan
    velodyne_msgs
  )

  ## Generate messages in the 'msg' folder
  add_message_files(FILES
    Information.msg
    Lane.msg
    Lanes.msg
    Object.msg
    Objects.msg
    Street.msg
    StreetMap.msg
    StreetPoint.msg
  )

  ## Generate added messages and services with any dependencies listed here
  generate_messages(DEPENDENCIES std_msgs nav_msgs geometry_msgs)

  ###################################
  ## catkin specific configuration ##
  ###################################
  ## The catkin_package macro generates cmake config files for your package
  ## Declare things to be passed to dependent projects
  ## INCLUDE_DIRS: uncomment this if you package contains header files
  ## LIBRARIES: libraries you create in this project that dependent projects also need
  ## CATKIN_DEPENDS: catkin_packages dependent projects also need
  ## DEPENDS: system dependencies of this project that dependent projects also need
  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_msgs message_runtime
    LIBRARIES ${ROCINANTE_NAME}
  )

endif(DEFINED CATKIN_DEVEL_PREFIX)
# // ------------------------------------------------------------------------------------ //
# // ------------------------------------- END ROS -------------------------------------- //
# // ------------------------------------------------------------------------------------ //

#ros directory
set(ROS_INC_DIR /opt/ros/kinetic)

#set include directories
set(INC_DIRS
  ${ROS_INC_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

#set lib directories
set(LIB_DIRS
  ${ROS_INC_DIR}/lib
)

#include directories
include_directories(${INC_DIRS})
link_directories(${LIB_DIRS})

# Source files
set(SOURCES
  src/ros_camera_reader.cpp
  src/ros_free_space_reader.cpp
  src/ros_lane_reader.cpp
  src/ros_laser_reader.cpp
  src/ros_objects_reader.cpp
  src/ros_street_reader.cpp
  src/ros_vehicle_reader.cpp
)

include_directories(include)

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(
  ${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY
  ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES}
)

#build settings
add_library(${ROCINANTE_NAME}
  ${SOURCES}
)

target_link_libraries(${ROCINANTE_NAME}
  ${catkin_LIBRARIES}
)

if(DEFINED CATKIN_DEVEL_PREFIX)
  add_dependencies(${ROCINANTE_NAME}
    ${ROCINANTE_NAME}_generate_messages_cpp
    ${ROCINANTE_NAME}_generate_messages
    ${ROCINANTE_NAME}_generate_messages_py
    ${ROCINANTE_NAME}_generate_messages_lisp
  )
endif(DEFINED CATKIN_DEVEL_PREFIX)
