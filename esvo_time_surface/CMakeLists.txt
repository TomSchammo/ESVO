cmake_minimum_required(VERSION 3.5)
project(esvo_time_surface)

# To be consistent with the configuration in esvo_core
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
set(CMAKE_CXX_FLAGS "-O3")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

set(ROS2_DEPENDENCIES
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
  "builtin_interfaces"
  # "rclcpp_components"
  # "event_camera_msgs"
  # "std_srvs"
)

foreach(pkg ${ROS2_DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()


find_package(OpenCV REQUIRED)

ament_auto_find_build_dependencies(REQUIRED ${ROS2_DEPENDENCIES})


# make the executable
add_executable(esvo_time_surface
    src/TimeSurface.cpp
    src/TimeSurface_node.cpp
)

target_include_directories(esvo_time_surface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# link the executable to the necessary libs
ament_target_dependencies(esvo_time_surface
  ${ROS2_DEPENDENCIES}
)

target_link_libraries(esvo_time_surface
   ${OpenCV_LIBRARIES}
   glog
)

# install executable
install(TARGETS esvo_time_surface
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
