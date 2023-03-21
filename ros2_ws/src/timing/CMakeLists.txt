cmake_minimum_required(VERSION 3.8)
project(timing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

add_executable(Seq04 src/Seq04.cpp)
ament_target_dependencies(Seq04 rclcpp std_msgs rclcpp custom_interfaces)
add_executable(Loop04 src/Loop04.cpp)
ament_target_dependencies(Loop04 rclcpp std_msgs rclcpp custom_interfaces)

target_include_directories(Seq04 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(Loop04 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(Seq04 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(Loop04 PUBLIC c_std_99 cxx_std_17)

install(TARGETS 
  Seq04
  Loop04
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
