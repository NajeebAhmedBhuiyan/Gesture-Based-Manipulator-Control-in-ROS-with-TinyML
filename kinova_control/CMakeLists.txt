cmake_minimum_required(VERSION 3.8)
project(kinova_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/KeyTrigger.srv"
)

add_executable(moveit_service src/service_kinova_control.cpp)
target_include_directories(moveit_service PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(moveit_service PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(moveit_service
  "moveit_ros_planning_interface"
  "rclcpp"
  "rosidl_default_generators"
)

rosidl_get_typesupport_target(
  cpp_typesupport_target 
  ${PROJECT_NAME} 
  "rosidl_typesupport_cpp"
)
target_link_libraries(moveit_service "${cpp_typesupport_target}")

install(TARGETS moveit_service
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/client_kinova_control.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS 
  scripts/client_kinova_control.py
  scripts/gesture_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()