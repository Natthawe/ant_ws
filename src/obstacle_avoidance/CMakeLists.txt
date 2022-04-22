cmake_minimum_required(VERSION 3.8)
project(obstacle_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# reading_laser
add_executable(reading_laser src/reading_laser.cpp)
ament_target_dependencies(reading_laser rclcpp std_msgs sensor_msgs)

# obstacle avoidance
add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
ament_target_dependencies(obstacle_avoidance rclcpp sensor_msgs geometry_msgs)

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  reading_laser
  obstacle_avoidance
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
