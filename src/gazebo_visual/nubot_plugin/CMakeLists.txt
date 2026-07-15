cmake_minimum_required(VERSION 3.8)
project(nubot_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(nubot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_library(nubot_plugin SHARED
  src/nubot_plugin.cc
)

add_library(ball_gazebo SHARED
  src/ball_gazebo.cc
)

target_include_directories(nubot_plugin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_include_directories(ball_gazebo PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(nubot_plugin
  gz-sim8
  std_msgs
  nubot_interfaces
  rclcpp
  builtin_interfaces
  gz-transport13
  gz-msgs10
)

ament_target_dependencies(ball_gazebo
  gz-sim8
  gz-transport13
  gz-msgs10
)

target_link_libraries(nubot_plugin
  gz-sim8::core
  nubot_interfaces::nubot_interfaces_include
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::core
  gz-msgs10::gz-msgs10
)

target_link_libraries(ball_gazebo
  gz-sim8::core
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::core
  gz-msgs10::gz-msgs10
)

install(TARGETS nubot_plugin ball_gazebo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

ament_package()
