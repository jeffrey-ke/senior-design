cmake_minimum_required(VERSION 3.8)
project(profiler_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#Find custom interface files
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs)
find_package(std_msgs)
find_package(geographic_msgs)
set(msg_files
  "msg/Pwm.msg"
  )
set(action_files
  "action/Profile.action"
  "action/Waypoint.action"
  )
set(srv_files
  "srv/GetDepth.srv"
  "srv/GetGnss.srv"
  "srv/GetOrientation.srv"
  "srv/SendKill.srv"
  "srv/SendPwm.srv"
  "srv/GetMinionStatus.srv"
  )
# Generating all custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  ${srv_files}
  
  DEPENDENCIES geographic_msgs geometry_msgs std_msgs
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
