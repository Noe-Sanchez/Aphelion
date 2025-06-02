cmake_minimum_required(VERSION 3.8)
project(aphelion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG 0.8.0 # Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
)
FetchContent_MakeAvailable(yaml-cpp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rplidar_ros REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(aruco_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(simple_kinematics_node src/simple_kinematics.cpp)
ament_target_dependencies(simple_kinematics_node rclcpp std_msgs geometry_msgs tf2_ros)

add_executable(asmc_node src/asmc.cpp)
ament_target_dependencies(asmc_node rclcpp std_msgs geometry_msgs tf2_ros nav_msgs tf2_geometry_msgs)

add_executable(formation_asmc_node src/formation_asmc.cpp)
ament_target_dependencies(formation_asmc_node rclcpp std_msgs geometry_msgs tf2_ros nav_msgs)

add_executable(odom_node src/odom.cpp)
ament_target_dependencies(odom_node rclcpp std_msgs geometry_msgs tf2_ros nav_msgs aruco_interfaces yaml-cpp ament_index_cpp)

add_executable(restamper_node src/restamper.cpp)
ament_target_dependencies(restamper_node std_msgs rclcpp sensor_msgs geometry_msgs tf2_ros nav_msgs) 

add_executable(guild_navigator_node src/guild_navigator.cpp)
ament_target_dependencies(guild_navigator_node rclcpp std_msgs geometry_msgs nav_msgs visualization_msgs)

add_executable(master_control_node src/master_control.cpp)
ament_target_dependencies(master_control_node rclcpp std_msgs lifecycle_msgs sensor_msgs)

#ament_python_install_package(${PROJECT_NAME})

#install(PROGRAMS
#  DESTINATION lib/${PROJECT_NAME}
#)

target_link_libraries(odom_node ${YAML_CPP_LIBRARIES})

install(TARGETS
  odom_node
  simple_kinematics_node
  asmc_node
  formation_asmc_node
  restamper_node
  guild_navigator_node
  master_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf   DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
