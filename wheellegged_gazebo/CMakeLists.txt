cmake_minimum_required(VERSION 3.10)
project(wheellegged_gazebo)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        gazebo_dev
        gazebo_ros_control
        hardware_interface
        realtime_tools
        wheellegged_sdk_sim
        )

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        CATKIN_DEPENDS
        roscpp
        gazebo_ros_control
        hardware_interface
        realtime_tools
        wheellegged_sdk_sim
        DEPENDS
)

###########
## Build ##
###########

link_directories(${CATKIN_DEVEL_PREFIX}/lib)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(wheellegged_hw_sim
        src/WheelleggedHWSim.cpp
        )

add_dependencies(wheellegged_hw_sim
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(wheellegged_hw_sim
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        wheellegged_sdk_sim
        )


#############
## Install ##
#############

install(TARGETS wheellegged_hw_sim
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

install(FILES wheellegged_hw_sim_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY config launch worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
