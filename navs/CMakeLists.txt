cmake_minimum_required(VERSION 3.8)
project(navs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(orbiter_bt REQUIRED)
find_package(geometry_msgs REQUIRED)

install(DIRECTORY
launch config maps rviz2
DESTINATION share/${PROJECT_NAME}
)

set(TARGET_DEPENDS
    rclcpp
    yaml-cpp
    tf2
    tf2_geometry_msgs
    orbiter_bt
    geometry_msgs
)


include_directories(include)

add_executable(set_amcl_init_pose src/set_amcl_init_pose.cpp)
add_executable(cmd_vel_interface src/cmd_vel_interface.cpp)
ament_target_dependencies(set_amcl_init_pose ${TARGET_DEPENDS})
ament_target_dependencies(cmd_vel_interface ${TARGET_DEPENDS})


install(TARGETS
  set_amcl_init_pose
  cmd_vel_interface
  DESTINATION lib/${PROJECT_NAME}
)

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
