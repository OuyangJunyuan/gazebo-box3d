cmake_minimum_required(VERSION 3.0.2)
project(dump_gt_box)
if (NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif ()
set(CMAKE_BUILD_TYPE "release")  #release debug 模式

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        tf
        gazebo_ros
        )
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
        INCLUDE_DIRS  ${GAZEBO_INCLUDE_DIRS}
        LIBRARIES dump_gt_box
        CATKIN_DEPENDS roscpp sensor_msgs gazebo_ros
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)


add_library(dump_gt_box src/plugin.cpp)
target_link_libraries(dump_gt_box
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        )

install(TARGETS dump_gt_box
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )


