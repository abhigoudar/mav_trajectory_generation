cmake_minimum_required(VERSION 3.5)
project(mav_visualization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

set(DEPENDENCIES
  rclcpp
  visualization_msgs
  tf2_eigen
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/marker_group.cpp
  src/leica_marker.cpp
  src/hexacopter_marker.cpp
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_dependencies(eigen3_cmake_module)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
