cmake_minimum_required(VERSION 3.8)
project(studica_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(studica_drivers REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

get_target_property(STUDICA_DRIVERS_TARGET_TYPE studica_drivers::studica_drivers TYPE)
set(STUDICA_CONTROL_WITH_HARDWARE TRUE)
if(STUDICA_DRIVERS_TARGET_TYPE STREQUAL "INTERFACE_LIBRARY")
  message(WARNING "studica_drivers built without VMXPi support. Building gamepad-only nodes in studica_ros2_control.")
  set(STUDICA_CONTROL_WITH_HARDWARE FALSE)
endif()

if(STUDICA_CONTROL_WITH_HARDWARE)
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/DutyCycleEncoderMsg.msg"
    "msg/EncoderMsg.msg"
    "msg/InitializeParams.msg"
    "srv/ControlImu.srv"
    "srv/SetData.srv"
  )
endif()

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "std_msgs"
  "std_srvs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)

set(studica_ros2_control_include_dirs
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(gamepad_component SHARED src/components/gamepad_component.cpp)
ament_target_dependencies(gamepad_component ${dependencies})
rclcpp_components_register_nodes(gamepad_component "studica_ros2_control::GamepadController")

set(studica_ros2_control_component_targets gamepad_component)
set(studica_ros2_control_export_libraries gamepad_component)
set(studica_ros2_control_executables)

add_executable(gamepad_teleop src/gamepad_teleop_main.cpp)
target_link_libraries(gamepad_teleop gamepad_component)
ament_target_dependencies(gamepad_teleop ${dependencies})
list(APPEND studica_ros2_control_executables gamepad_teleop)

if(STUDICA_CONTROL_WITH_HARDWARE)
  add_library(cobra_component SHARED src/components/cobra_component.cpp)
  ament_target_dependencies(cobra_component ${dependencies})
  target_link_libraries(cobra_component studica_drivers::studica_drivers)
  rclcpp_components_register_nodes(cobra_component "studica_ros2_control::Cobra")

  add_library(dc_encoder_component SHARED src/components/dc_encoder_component.cpp)
  ament_target_dependencies(dc_encoder_component ${dependencies})
  target_link_libraries(dc_encoder_component studica_drivers::studica_drivers)
  rclcpp_components_register_nodes(dc_encoder_component "studica_ros2_control::DutyCycleEncoder")

  add_library(diff_drive_component SHARED src/components/diff_drive_component.cpp)
  ament_target_dependencies(diff_drive_component ${dependencies})
  target_link_libraries(diff_drive_component studica_drivers::studica_drivers)
  rclcpp_components_register_nodes(diff_drive_component "studica_ros2_control::DiffDrive")

  add_library(diff_drive_odometry SHARED src/components/diff_drive_odometry.cpp)
  ament_target_dependencies(diff_drive_odometry ${dependencies})
  rclcpp_components_register_nodes(diff_drive_odometry "studica_ros2_control::DiffOdometry")

  add_library(dio_component SHARED src/components/dio_component.cpp)
  ament_target_dependencies(dio_component ${dependencies})
  target_link_libraries(dio_component studica_drivers::studica_drivers)
  rclcpp_components_register_nodes(dio_component "studica_ros2_control::DIO")

  add_library(encoder_component SHARED src/components/encoder_component.cpp)
  ament_target_dependencies(encoder_component ${dependencies})
  target_link_libraries(encoder_component studica_drivers::studica_drivers)
  rclcpp_components_register_nodes(encoder_component "studica_ros2_control::Encoder")

  add_library(imu_component SHARED src/components/imu_component.cpp)
  ament_target_dependencies(imu_component ${dependencies})
  target_link_libraries(imu_component studica_drivers::studica_drivers)
  rclcpp_components_register_nodes(imu_component "studica_ros2_control::Imu")

  add_library(mecanum_drive_component SHARED src/components/mecanum_drive_component.cpp)
  ament_target_dependencies(mecanum_drive_component ${dependencies})
  target_link_libraries(mecanum_drive_component studica_drivers::studica_drivers)
  rclcpp_components_register_nodes(mecanum_drive_component "studica_ros2_control::MecanumDrive")

  add_library(mecanum_drive_odometry SHARED src/components/mecanum_drive_odometry.cpp)
  ament_target_dependencies(mecanum_drive_odometry ${dependencies})

  add_library(servo_component SHARED src/components/servo_component.cpp)
  ament_target_dependencies(servo_component ${dependencies})
  target_link_libraries(servo_component studica_drivers::studica_drivers)
  rclcpp_components_register_nodes(servo_component "studica_ros2_control::Servo")

  add_library(sharp_component SHARED src/components/sharp_component.cpp)
  ament_target_dependencies(sharp_component ${dependencies})
  target_link_libraries(sharp_component studica_drivers::studica_drivers)
  rclcpp_components_register_nodes(sharp_component "studica_ros2_control::Sharp")

  add_library(titan_component SHARED src/components/titan_component.cpp)
  ament_target_dependencies(titan_component ${dependencies})
  target_link_libraries(titan_component studica_drivers::studica_drivers)
  rclcpp_components_register_nodes(titan_component "studica_ros2_control::Titan")

  add_library(ultrasonic_component SHARED src/components/ultrasonic_component.cpp)
  ament_target_dependencies(ultrasonic_component ${dependencies})
  target_link_libraries(ultrasonic_component studica_drivers::studica_drivers)
  rclcpp_components_register_nodes(ultrasonic_component "studica_ros2_control::Ultrasonic")

  target_link_libraries(diff_drive_component diff_drive_odometry)
  target_link_libraries(mecanum_drive_component mecanum_drive_odometry)

  list(APPEND studica_ros2_control_component_targets
    cobra_component
    dc_encoder_component
    diff_drive_component
    diff_drive_odometry
    dio_component
    encoder_component
    imu_component
    mecanum_drive_component
    mecanum_drive_odometry
    servo_component
    sharp_component
    titan_component
    ultrasonic_component
  )

  list(APPEND studica_ros2_control_export_libraries
    cobra_component
    dc_encoder_component
    diff_drive_component
    diff_drive_odometry
    dio_component
    encoder_component
    imu_component
    mecanum_drive_component
    servo_component
    sharp_component
    titan_component
    ultrasonic_component
  )

  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

  add_executable(manual_composition src/manual_composition.cpp)
  target_link_libraries(manual_composition
    cobra_component
    dc_encoder_component
    diff_drive_component
    diff_drive_odometry
    dio_component
    encoder_component
    gamepad_component
    imu_component
    mecanum_drive_component
    mecanum_drive_odometry
    servo_component
    sharp_component
    titan_component
    ultrasonic_component
    studica_drivers::studica_drivers
  )
  ament_target_dependencies(manual_composition ${dependencies})
  target_link_libraries(manual_composition "${cpp_typesupport_target}")
  list(APPEND studica_ros2_control_executables manual_composition)
endif()

foreach(target
  ${studica_ros2_control_component_targets}
  ${studica_ros2_control_executables}
)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_include_directories(${target} PUBLIC ${studica_ros2_control_include_dirs})
  if(STUDICA_CONTROL_WITH_HARDWARE)
    target_include_directories(${target} PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)
  endif()
endforeach()

install(TARGETS
  ${studica_ros2_control_component_targets}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${studica_ros2_control_executables}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
if(STUDICA_CONTROL_WITH_HARDWARE)
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/
    DESTINATION include/${PROJECT_NAME}
  )
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  option(STUDICA_STRICT_LINT "Enable full strict lint checks" OFF)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.flake8")
  if(NOT STUDICA_STRICT_LINT)
    # Legacy codebase: enable lint infrastructure first, then tighten checks incrementally.
    set(AMENT_LINT_AUTO_EXCLUDE
      ament_cmake_copyright
      ament_cmake_cpplint
      ament_cmake_uncrustify
    )
  endif()
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${studica_ros2_control_export_libraries})
ament_export_dependencies(${dependencies})

ament_package()
