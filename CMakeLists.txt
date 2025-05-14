cmake_minimum_required(VERSION 3.8)
project(aphelion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rp_lidar_ros REQUIRED)
find_package(slam_toolbox REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(simple_kinematics_node src/simple_kinematics.cpp)
ament_target_dependencies(simple_kinematics_node rclcpp std_msgs geometry_msgs tf2_ros)

add_executable(asmc_node src/asmc.cpp)
ament_target_dependencies(asmc_node rclcpp std_msgs geometry_msgs tf2_ros)

add_executable(formation_asmc_node src/formation_asmc.cpp)
ament_target_dependencies(formation_asmc_node rclcpp std_msgs geometry_msgs tf2_ros)

add_executable(odom_node src/odom.cpp)
ament_target_dependencies(odom_node rclcpp std_msgs geometry_msgs tf2_ros)

#ament_python_install_package(${PROJECT_NAME})

#install(PROGRAMS
#  DESTINATION lib/${PROJECT_NAME}
#)

install(TARGETS
  odom_node
  simple_kinematics_node
  asmc_node
  formation_asmc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf   DESTINATION share/${PROJECT_NAME})

ament_package()
