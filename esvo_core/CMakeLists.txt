cmake_minimum_required(VERSION 3.5)
project(esvo_core)

# explicitly set std=c++14 to remove errors from pcl library
# C++20 is required for std::format
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O3")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dvs_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
# find_package(pcl_ros REQUIRED)  # Not needed, using pcl_conversions
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(minkindr REQUIRED)
find_package(minkindr_conversions REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

set(ROS2_DEPENDENCIES
  rclcpp
  dvs_msgs
  cv_bridge
  image_geometry
  image_transport
  pcl_conversions
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  builtin_interfaces
  message_filters
  minkindr
  minkindr_conversions
)

###########
## Build ##
###########

set(HEADERS
  include/esvo_core/core/DepthFusion.h
  include/esvo_core/core/DepthRegularization.h
  include/esvo_core/core/DepthProblem.h
  include/esvo_core/core/DepthProblemSolver.h
  include/esvo_core/core/EventBM.h
  include/esvo_core/core/RegProblemLM.h
  include/esvo_core/core/RegProblemSolverLM.h
  include/esvo_core/core/EventMatcher.h
  include/esvo_core/optimization/OptimizationFunctor.h
  include/esvo_core/container/CameraSystem.h
  include/esvo_core/container/DepthPoint.h
  include/esvo_core/container/EventMatchPair.h
  include/esvo_core/container/SmartGrid.h
  include/esvo_core/container/DepthMap.h
  include/esvo_core/container/ResidualItem.h
  include/esvo_core/container/TimeSurfaceObservation.h
  include/esvo_core/tools/Visualization.h
  include/esvo_core/tools/utils.h
  include/esvo_core/tools/TicToc.h
  include/esvo_core/tools/sobel.h
  include/esvo_core/tools/cayley.h
  include/esvo_core/tools/params_helper.h)

set(SOURCES
  src/core/DepthFusion.cpp
  src/core/DepthRegularization.cpp
  src/core/DepthProblem.cpp
  src/core/DepthProblemSolver.cpp
  src/core/EventBM.cpp
  src/core/RegProblemLM.cpp
  src/core/RegProblemSolverLM.cpp
  src/core/EventMatcher.cpp
  src/container/CameraSystem.cpp
  src/container/DepthPoint.cpp
  src/container/ResidualItem.cpp
  src/tools/Visualization.cpp
  src/tools/sobel.cpp
  src/tools/cayley.cpp)

# Library
add_library(${PROJECT_NAME}_LIB ${SOURCES} ${HEADERS})
target_include_directories(${PROJECT_NAME}_LIB PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_LIB ${ROS2_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_LIB
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
  glog
)

# Node esvo_Mapping
add_executable(esvo_Mapping
  src/esvo_MappingNode.cpp
  src/esvo_Mapping.cpp
  include/esvo_core/esvo_Mapping.h)
target_include_directories(esvo_Mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(esvo_Mapping ${ROS2_DEPENDENCIES})
target_link_libraries(esvo_Mapping
  ${PROJECT_NAME}_LIB
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
  glog
)

# Node esvo_Tracking
add_executable(esvo_Tracking
  src/esvo_TrackingNode.cpp
  src/esvo_Tracking.cpp
  include/esvo_core/esvo_Tracking.h)
target_include_directories(esvo_Tracking PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(esvo_Tracking ${ROS2_DEPENDENCIES})
target_link_libraries(esvo_Tracking
  ${PROJECT_NAME}_LIB
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
  glog
)

# Node esvo_MVStereo
add_executable(esvo_MVStereo
  src/esvo_MVStereoNode.cpp
  src/esvo_MVStereo.cpp
  include/esvo_core/esvo_MVStereo.h)
target_include_directories(esvo_MVStereo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(esvo_MVStereo ${ROS2_DEPENDENCIES})
target_link_libraries(esvo_MVStereo
  ${PROJECT_NAME}_LIB
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
  glog
)

# Install
install(TARGETS
  ${PROJECT_NAME}_LIB
  esvo_Mapping
  esvo_Tracking
  esvo_MVStereo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
)

install(DIRECTORY calib/
  DESTINATION share/${PROJECT_NAME}/calib
)

# Install rviz and perspective files
install(FILES
  esvo_system.rviz
  esvo_mvstereo.rviz
  esvo_system_DSEC.rviz
  esvo_system.perspective
  esvo_mvstereo.perspective
  esvo_system_DSEC.perspective
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/sync_timer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
