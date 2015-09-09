cmake_minimum_required(VERSION 2.8.3)
project(robotis_op_gazebo)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  gazebo_ros  
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp
    rospy
    std_msgs
    sensor_msgs
    gazebo_ros 
    hector_gazebo_plugins
  DEPENDS 
    gazebo
)


## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Specify additional locations of header files
set(HEADERS
    include/${PROJECT_NAME}/gazebo_walking.h
    include/${PROJECT_NAME}/gazebo_walking_node.h
    include/${PROJECT_NAME}/math/Matrix.h
    include/${PROJECT_NAME}/math/Plane.h
    include/${PROJECT_NAME}/math/Point.h
    include/${PROJECT_NAME}/math/Vector.h
)

set(SOURCES
    src/gazebo_walking.cpp
    src/gazebo_walking_node.cpp
    src/math/Matrix.cpp
    src/math/Plane.cpp
    src/math/Point.cpp
    src/math/Vector.cpp
)


## Declare a cpp library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

## Declare a cpp executable
add_executable(robotis_op_gazebo_walking_node src/gazebo_walking_node.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(robotis_op_gazebo_walking_node ${PROJECT_NAME} )

