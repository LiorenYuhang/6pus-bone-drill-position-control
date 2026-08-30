cmake_minimum_required(VERSION 3.8)
project(h30_imu)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TimedImu.msg"
  "msg/RelativeAttitude.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

add_library(h30_protocol src/h30_protocol.cpp src/source_capture_writer.cpp)
target_compile_features(h30_protocol PUBLIC cxx_std_17)
target_include_directories(h30_protocol PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
add_executable(h30_imu_node src/h30_imu_node.cpp)
target_compile_features(h30_imu_node PUBLIC cxx_std_17)
target_link_libraries(h30_imu_node h30_protocol)
ament_target_dependencies(h30_imu_node rclcpp sensor_msgs geometry_msgs diagnostic_msgs tf2_ros)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(h30_imu_node "${cpp_typesupport_target}")

add_executable(h30_relative_attitude_node src/h30_relative_attitude_node.cpp)
target_compile_features(h30_relative_attitude_node PUBLIC cxx_std_17)
ament_target_dependencies(h30_relative_attitude_node rclcpp sensor_msgs geometry_msgs std_msgs diagnostic_msgs)
target_link_libraries(h30_relative_attitude_node "${cpp_typesupport_target}")

install(TARGETS h30_protocol h30_imu_node h30_relative_attitude_node ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/imu_csv_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_csv_visualizer
)
install(PROGRAMS
  scripts/j9_e1a_platform_orientation_calibrator.py
  scripts/j9_e1b_r2_nomotion_qualification.py
  scripts/j9_e1b_r4_active_load_qualification.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_h30_protocol test/test_h30_protocol.cpp)
  ament_add_gtest(test_source_capture_writer test/test_source_capture_writer.cpp)
  target_link_libraries(test_source_capture_writer h30_protocol)
  target_link_libraries(test_h30_protocol h30_protocol)
  ament_add_gtest(test_j9_b_timed_interfaces test/test_j9_b_timed_interfaces.cpp)
  ament_target_dependencies(test_j9_b_timed_interfaces rclcpp sensor_msgs geometry_msgs std_msgs)
  target_link_libraries(test_j9_b_timed_interfaces "${cpp_typesupport_target}")
  ament_add_pytest_test(test_j9_b_h30_integration test/test_j9_b_h30_integration.py)
  ament_add_pytest_test(test_j9_e1a_platform_orientation_calibrator test/test_j9_e1a_platform_orientation_calibrator.py)
  ament_add_pytest_test(test_j9_e1b_r4a_qualification test/test_j9_e1b_r4a_qualification.py)
endif()
ament_export_include_directories(include)
ament_export_libraries(h30_protocol)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
