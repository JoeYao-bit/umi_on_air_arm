cmake_minimum_required(VERSION 3.17)
project(umi_on_air_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(umi_arm_msg REQUIRED)
include_directories(${umi_arm_msg_INCLUDE_DIR})


set(OpenCV_INCLUDE_DIRS "/usr/include/opencv4")
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)


add_executable(arm_ik_node src/IK_node.cpp src/arm_3dof.cpp)

ament_target_dependencies(arm_ik_node
  rclcpp
  umi_arm_msg
)

install(TARGETS
  arm_ik_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()