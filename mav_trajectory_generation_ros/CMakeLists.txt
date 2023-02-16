

cmake_minimum_required(VERSION 3.5)
project(mav_trajectory_generation_ros)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(mav_planning_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(mav_trajectory_generation REQUIRED)
find_package(mav_visualization REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

set(DEPENDENCIES
  rclcpp
  geometry_msgs
  mav_msgs
  mav_planning_msgs
  sensor_msgs
  nav_msgs
  tf2_eigen
  std_srvs
  visualization_msgs
  mav_trajectory_generation
  mav_visualization
  trajectory_msgs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/feasibility_analytic.cpp
  src/feasibility_base.cpp
  src/feasibility_recursive.cpp
  src/feasibility_sampling.cpp
  src/input_constraints.cpp
  src/ros_conversions.cpp
  src/ros_visualization.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

add_executable(trajectory_sampler_node 
               src/trajectory_sampler_node.cpp)
ament_target_dependencies(trajectory_sampler_node ${DEPENDENCIES})
target_link_libraries(trajectory_sampler_node ${PROJECT_NAME} glog)

install(TARGETS trajectory_sampler_node
   RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(eigen3_cmake_module)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_package()

