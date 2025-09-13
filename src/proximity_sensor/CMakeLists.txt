cmake_minimum_required(VERSION 3.8)
project(proximity_sensor)
include(../CMakeCommon.txt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(SOURCE_FILES
  src/list_distance_source.cpp
)

add_executable(node_${PROJECT_NAME}  
  src/node_proximity_sensor.cpp
  ${SOURCE_FILES}
)

ament_target_dependencies(node_${PROJECT_NAME} rclcpp std_msgs)

target_include_directories(node_${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS node_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_add_gmock(test_${PROJECT_NAME}
    test/test_list_distance_source.cpp
    ${SOURCE_FILES}
  )

  target_include_directories(test_${PROJECT_NAME} PRIVATE
    src
  )

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
