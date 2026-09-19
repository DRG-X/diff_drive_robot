cmake_minimum_required(VERSION 3.8)
project(diff_drive_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# --- LoRa bridge ---
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoverTelemetry.msg"
  DEPENDENCIES std_msgs
)

# Shared protocol/simulation code, importable as `lora_bridge`.
ament_python_install_package(lora_bridge)

# Node entry points, installed without the .py so `ros2 run` reads naturally.
install(PROGRAMS lora_bridge/lora_receiver_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME lora_receiver)
install(PROGRAMS lora_bridge/fake_rover_tx.py
  DESTINATION lib/${PROJECT_NAME} RENAME fake_rover_tx)

install(
  DIRECTORY launch urdf config worlds rviz firmware
  DESTINATION share/${PROJECT_NAME}
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

  # LoRa protocol tests: pure python, no hardware or ROS graph needed.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_lora_protocol test/test_lora_protocol.py)
  # Checks the ESP32 firmware and the base station encode identical bytes.
  # Skips itself if no C compiler is installed.
  ament_add_pytest_test(test_firmware_parity test/test_firmware_parity.py)
endif()

ament_package()
