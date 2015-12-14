cmake_minimum_required(VERSION 2.8.3)
project(pandora_tld)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  image_transport
  sensor_msgs
  dynamic_reconfigure
  geometry_msgs
)

find_package(OpenCV REQUIRED)
find_package(open_tld REQUIRED)

include_directories(include
  ${open_tld_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(
  config/predator_cfg.cfg
)

catkin_package(
 #INCLUDE_DIRS include
  DEPENDS
    OpenCV
  CATKIN_DEPENDS
    roscpp
    cv_bridge
    image_transport
    sensor_msgs
    open_tld
    geometry_msgs
)

add_executable(predator_node
      src/predator_node.cpp
      src/predator.cpp)

target_link_libraries(predator_node
  ${open_tld_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES})

add_dependencies(predator_node
    ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

