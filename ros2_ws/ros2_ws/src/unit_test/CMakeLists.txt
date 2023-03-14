cmake_minimum_required(VERSION 3.8)
project(unit_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(asdfr_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgcodecs imgproc videoio)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

add_executable(set_point_pub src/set_point_pub.cpp)
ament_target_dependencies(set_point_pub rclcpp asdfr_interfaces)
add_executable(light_setpoint src/light_setpoint.cpp)
ament_target_dependencies(light_setpoint rclcpp asdfr_interfaces)
add_executable(controller src/controller.cpp)
ament_target_dependencies(controller rclcpp asdfr_interfaces)


target_include_directories(set_point_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(set_point_pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(light_setpoint PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(light_setpoint PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS set_point_pub
  light_setpoint 
  controller 
  DESTINATION lib/${PROJECT_NAME})

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

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

