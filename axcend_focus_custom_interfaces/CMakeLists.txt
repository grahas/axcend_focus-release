cmake_minimum_required(VERSION 3.8)
project(axcend_focus_custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/CartridgeStatus.msg"
  "msg/CartridgeDetectorStatus.msg"
  "msg/CartridgeOvenStatus.msg"
  "msg/PumpStatus.msg"
  "msg/ValveStatus.msg"
  "msg/PumpControlMode.msg"
  "msg/MethodInjectionType.msg"
)

set(action_files
    "action/CartridgeOvenPreheat.action"
    "action/Method.action"
    "action/Pump.action"
    "action/PumpMove.action"
    "action/PumpPositioning.action"
    "action/PumpService.action"
    "action/PumpSystemStabilization.action"
    "action/PumpVoidRemoval.action"
    "action/SolventExchange.action"
    "action/ValveRotate.action"
)

set(srv_files
  "srv/CartridgeMemoryReadWrite.srv"
  "srv/SystemParametersUpdate.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    ${action_files}
    DEPENDENCIES 
      action_msgs 
      std_msgs 
      builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

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
