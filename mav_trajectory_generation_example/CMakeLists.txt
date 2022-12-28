cmake_minimum_required(VERSION 3.5)
project(mav_trajectory_generation_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

############
# BINARIES #
############
cs_add_executable(trajectory_generation_example
        src/example_planner.cc
        src/example_planner_node.cc
)

cs_add_executable(trajectory_generation_6dof_example
        src/example_planner_6dof.cc
        src/example_planner_6dof_node.cc
)

##########
# EXPORT #
##########
cs_install()
cs_export()
