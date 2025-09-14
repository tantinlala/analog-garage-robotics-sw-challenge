cmake_minimum_required(VERSION 3.8)
project(speed_limiter)
include(../CMakeCommon.txt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(state_machine REQUIRED)

set(INCLUDE_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(node_${PROJECT_NAME}
  src/node_speed_limiter.cpp
  src/speed_limiter_states.cpp
  src/speed_state_publisher.cpp
)

ament_target_dependencies(node_${PROJECT_NAME} rclcpp std_msgs state_machine)
target_include_directories(node_${PROJECT_NAME} PUBLIC
  ${INCLUDE_DIRS}
)

install(TARGETS node_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

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

  set(TEST_FILES
    test/test_estopped_state.cpp
    test/test_not_estopped_state.cpp
  )

  ament_add_gmock(test_${PROJECT_NAME}
    ${TEST_FILES}
    src/speed_limiter_states.cpp
  )

  target_include_directories(test_${PROJECT_NAME} PRIVATE
    ${INCLUDE_DIRS}
  )

  target_include_directories(test_${PROJECT_NAME} PRIVATE
    src
  )

  ament_target_dependencies(test_${PROJECT_NAME} state_machine)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
