cmake_minimum_required(VERSION 3.8)
project(webcam_downsampling)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

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

find_package(OpenCV REQUIRED)
find_package(std_filesystem REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(image_publisher src/image_publisher.cpp)
ament_target_dependencies(image_publisher rclcpp sensor_msgs std_msgs cv_bridge)
target_link_libraries(image_publisher ${OpenCV_LIBS})

add_executable(image_saver src/image_saver.cpp)
ament_target_dependencies(image_saver rclcpp sensor_msgs std_msgs cv_bridge)
target_link_libraries(image_saver ${OpenCV_LIBS})

install(TARGETS
  image_publisher
  image_saver
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
