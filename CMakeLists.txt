cmake_minimum_required(VERSION 3.22)
project(lite6_moveit_demos)

find_package(ament_cmake REQUIRED)

add_subdirectory(motion_planning_scripts)
add_subdirectory(notebook_demos)

ament_package()
