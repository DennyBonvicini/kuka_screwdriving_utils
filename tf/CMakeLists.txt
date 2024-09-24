cmake_minimum_required(VERSION 3.8)
project(tf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TfPublish.srv"
  "srv/ComputeCenter.srv"
  DEPENDENCIES geometry_msgs
)

# Installa script
install(
  PROGRAMS
    src/tf_publish_service.cpp
    src/compute_center_service.cpp
  DESTINATION lib/${PROJECT_NAME}
  )
# Create the service node
add_executable(tf_publish_service src/tf_publish_service.cpp)
add_executable(compute_center_service src/compute_center_service.cpp)

# Link the generated service interface
rosidl_target_interfaces(
	tf_publish_service
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )
  
rosidl_target_interfaces(
	compute_center_service
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )

ament_target_dependencies(
        tf_publish_service
        tf2_ros
        tf2_msgs
        geometry_msgs
        rclcpp
        std_msgs
        )
        
ament_target_dependencies(
        compute_center_service
        tf2_ros
        tf2_msgs
        geometry_msgs
        rclcpp
        std_msgs
        )

install(TARGETS
        tf_publish_service
        compute_center_service
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
