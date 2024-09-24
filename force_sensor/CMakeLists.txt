cmake_minimum_required(VERSION 3.8)
project(force_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED) 

#create the service interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CheckTouch.srv"
)

# Installazione dello script Python
install(PROGRAMS
  scripts/ft_ati_publisher.py  
  DESTINATION lib/${PROJECT_NAME}
)

# Include the generated headers
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Create the service node
add_executable(check_touch_service src/check_touch_service.cpp)

ament_target_dependencies(
	check_touch_service
 	rclcpp
	std_msgs
	rclcpp_action
	control_msgs
	)

# Link the generated service interface
rosidl_target_interfaces(check_touch_service
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )
  
install(TARGETS
	check_touch_service
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
