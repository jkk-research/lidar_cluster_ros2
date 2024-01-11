cmake_minimum_required(VERSION 3.8)
project(lidar_cluster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Compile as C++17 and with optimization level O2
add_compile_options(-std=c++17 -O2)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED) 

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(INCLUDE_DIRS include 
  ${ament_cmake_INCLUDE_DIRS} 
  ${rclcpp_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS} 
  ${pcl_conversions_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
  include
)
include_directories(${INCLUDE_DIRS})

set(ament_dependencies
  pcl_conversions
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs  
  PCL
)

add_executable(dbscan_spatial src/dbscan_spatial.cpp)
target_link_libraries(dbscan_spatial ${PCL_LIBRARIES})
ament_target_dependencies(dbscan_spatial  ${ament_dependencies} )
target_compile_features(dbscan_spatial PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(dbscan_grid src/dbscan_grid.cpp)
target_link_libraries(dbscan_grid ${PCL_LIBRARIES})
ament_target_dependencies(dbscan_grid  ${ament_dependencies} )
target_compile_features(dbscan_grid PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(dblane_spatial src/dblane_spatial.cpp)
target_link_libraries(dblane_spatial ${PCL_LIBRARIES})
ament_target_dependencies(dblane_spatial  ${ament_dependencies} )
target_compile_features(dblane_spatial PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(eucledian_spatial src/eucledian_spatial.cpp)
target_link_libraries(eucledian_spatial ${PCL_LIBRARIES})
ament_target_dependencies(eucledian_spatial  ${ament_dependencies} )
target_compile_features(eucledian_spatial PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(dblane_f1s src/dblane_f1s.cpp)
target_link_libraries(dblane_f1s ${PCL_LIBRARIES})
ament_target_dependencies(dblane_f1s  ${ament_dependencies} )
target_compile_features(dblane_f1s PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME})

install(TARGETS 
  dbscan_spatial
  dbscan_grid
  dblane_spatial
  eucledian_spatial
  dblane_f1s
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
