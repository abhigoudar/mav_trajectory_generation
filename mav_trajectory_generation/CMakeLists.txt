cmake_minimum_required(VERSION 3.5)
project(mav_trajectory_generation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

find_package(ament_cmake REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)
pkg_check_modules(NLopt REQUIRED)

set(DEPENDENCIES
  mav_msgs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIR}
)
#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  src/motion_defines.cpp
  src/polynomial.cpp
  src/segment.cpp
  src/timing.cpp
  src/trajectory.cpp
  src/trajectory_sampling.cpp
  src/vertex.cpp
  src/io.cpp
  src/rpoly/rpoly_ak1.cpp
)
# Link against yaml-cpp.
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} 
  yaml-cpp
  ${NLOPT_LIBRARIES}
  glog)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

############
# BINARIES #
############
add_executable(polynomial_timing_evaluation
  src/polynomial_timing_evaluation.cpp
)
target_link_libraries(polynomial_timing_evaluation ${PROJECT_NAME})
install(TARGETS polynomial_timing_evaluation
   RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)
#########
# TESTS #
#########

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

# catkin_add_gtest(test_polynomial
#   test/test_polynomial.cpp
# )
# target_link_libraries(test_polynomial ${PROJECT_NAME} ${catkin_LIBRARIES})

# catkin_add_gtest(test_polynomial_optimization
#   test/test_polynomial_optimization.cpp
# )
# target_link_libraries(test_polynomial_optimization ${PROJECT_NAME} ${catkin_LIBRARIES})

##########
# EXPORT #
##########
ament_export_dependencies(eigen3_cmake_module)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(NLopt yaml-cpp glog)

ament_package()
