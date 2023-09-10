cmake_minimum_required(VERSION 3.8)
project(px4_manipulation)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(manipulation_msgs REQUIRED)
find_package(px4_msgs REQUIRED)


include_directories(
  include
)

add_executable(talker
  src/main.cpp
  src/px4_manipulation.cpp)
ament_target_dependencies(talker rclcpp std_msgs geometry_msgs px4_msgs manipulation_msgs)
target_include_directories(talker PUBLIC "${Eigen3_INCLUDE_DIRS}")
target_link_libraries(talker)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  Tools/rviz_targetpose_marker.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
