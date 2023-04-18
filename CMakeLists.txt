cmake_minimum_required(VERSION 3.5)
project(gary_chassis)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(gary_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(mecanum_chassis_solver src/mecanum_chassis_solver.cpp src/utils/mecanum_kinematics.cpp)

target_include_directories(mecanum_chassis_solver PRIVATE include)

ament_target_dependencies(
        mecanum_chassis_solver
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        std_msgs
        nav_msgs
        control_msgs
        geometry_msgs
        diagnostic_msgs
)


add_library(mecanum_chassis_solver_lib SHARED
        src/mecanum_chassis_solver.cpp src/utils/mecanum_kinematics.cpp)

target_include_directories(mecanum_chassis_solver_lib PRIVATE include)

ament_target_dependencies(
        mecanum_chassis_solver_lib
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        std_msgs
        control_msgs
        nav_msgs
        geometry_msgs
        diagnostic_msgs
)

rclcpp_components_register_nodes(mecanum_chassis_solver_lib "gary_chassis::MecanumChassisSolver")


add_executable(chassis_teleop src/chassis_teleop.cpp src/utils/first_order_filter.cpp)

target_include_directories(chassis_teleop PRIVATE include)

ament_target_dependencies(
        chassis_teleop
        rclcpp
        diagnostic_msgs
        lifecycle_msgs
        rclcpp_lifecycle
        rclcpp_components
        gary_msgs
        geometry_msgs
        control_msgs
)


add_library(chassis_teleop_lib SHARED src/chassis_teleop.cpp src/utils/first_order_filter.cpp)

target_include_directories(chassis_teleop_lib PRIVATE include)

ament_target_dependencies(
        chassis_teleop_lib
        rclcpp
        diagnostic_msgs
        lifecycle_msgs
        rclcpp_lifecycle
        rclcpp_components
        control_msgs
        gary_msgs
        geometry_msgs
)

rclcpp_components_register_nodes(chassis_teleop_lib "gary_chassis::ChassisTeleop")


add_executable(chassis_autonomous src/chassis_autonomous.cpp src/utils/first_order_filter.cpp)

target_include_directories(chassis_autonomous PRIVATE include)

ament_target_dependencies(
        chassis_autonomous
        rclcpp
        diagnostic_msgs
        lifecycle_msgs
        rclcpp_lifecycle
        rclcpp_components
        gary_msgs
        geometry_msgs
        control_msgs
        nav_msgs
)


add_library(chassis_autonomous_lib SHARED src/chassis_autonomous.cpp src/utils/first_order_filter.cpp)

target_include_directories(chassis_autonomous_lib PRIVATE include)

ament_target_dependencies(
        chassis_autonomous_lib
        rclcpp
        diagnostic_msgs
        lifecycle_msgs
        rclcpp_lifecycle
        rclcpp_components
        control_msgs
        gary_msgs
        geometry_msgs
        nav_msgs
)

rclcpp_components_register_nodes(chassis_autonomous_lib "gary_chassis::ChassisAutonomous")


add_executable(chassis_power_control src/chassis_power_control.cpp)

target_include_directories(chassis_power_control PRIVATE include)

ament_target_dependencies(
        chassis_power_control
        rclcpp
        diagnostic_msgs
        lifecycle_msgs
        rclcpp_lifecycle
        rclcpp_components
        gary_msgs
        geometry_msgs
)


add_library(chassis_power_control_lib SHARED src/chassis_power_control.cpp)

target_include_directories(chassis_power_control_lib PRIVATE include)

ament_target_dependencies(
        chassis_power_control_lib
        rclcpp
        diagnostic_msgs
        lifecycle_msgs
        rclcpp_lifecycle
        rclcpp_components
        gary_msgs
        geometry_msgs
)

rclcpp_components_register_nodes(chassis_power_control_lib "gary_chassis::ChassisPowerControl")


add_executable(omni_chassis_solver src/omni_chassis_solver.cpp src/utils/omni_kinematics.cpp)

target_include_directories(omni_chassis_solver PRIVATE include)

ament_target_dependencies(
        omni_chassis_solver
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        control_msgs
        nav_msgs
        geometry_msgs
        std_msgs
        geometry_msgs
        diagnostic_msgs
)


add_library(omni_chassis_solver_lib SHARED
        src/omni_chassis_solver.cpp src/utils/omni_kinematics.cpp)

target_include_directories(omni_chassis_solver_lib PRIVATE include)

ament_target_dependencies(
        omni_chassis_solver_lib
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        control_msgs
        nav_msgs
        std_msgs
        geometry_msgs
        diagnostic_msgs
)

rclcpp_components_register_nodes(omni_chassis_solver_lib "gary_chassis::OmniChassisSolver")

add_executable(omni_odom src/omni_odometry.cpp src/utils/omni_kinematics.cpp)

target_include_directories(omni_odom PRIVATE include)

ament_target_dependencies(
        omni_odom
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        control_msgs
        nav_msgs
        std_msgs
        geometry_msgs
        diagnostic_msgs
)

add_library(omni_odom_lib SHARED src/omni_odometry.cpp src/utils/omni_kinematics.cpp)

target_include_directories(omni_odom_lib PRIVATE include)

ament_target_dependencies(
        omni_odom_lib
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        control_msgs
        nav_msgs
        geometry_msgs
        std_msgs
        diagnostic_msgs
)

rclcpp_components_register_nodes(omni_odom_lib "gary_chassis::OmniOdom")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS mecanum_chassis_solver chassis_teleop chassis_autonomous chassis_power_control omni_chassis_solver omni_odom
        DESTINATION lib/${PROJECT_NAME}
        )

install(TARGETS mecanum_chassis_solver_lib chassis_teleop_lib chassis_autonomous_lib chassis_power_control_lib omni_chassis_solver_lib omni_odom_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_export_dependencies(
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        gary_msgs
        std_msgs
        geometry_msgs
        diagnostic_msgs
        control_msgs
        nav_msgs
        sensor_msgs
)

ament_package()
