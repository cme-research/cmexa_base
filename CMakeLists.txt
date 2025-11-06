cmake_minimum_required(VERSION 3.8)
project(cmexa_base LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES  "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        cmeresearch_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
          "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(cmeresearch_msgs)

## COMPILE
add_library(
        cmexa_base
        SHARED
        src/mecanumbot_system.cpp
        src/diffbot_system.cpp
)
target_compile_features(cmexa_base PUBLIC cxx_std_17)
target_include_directories(cmexa_base PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/cmexa_base>
)

#ament_target_dependencies(
#        cmexa_base PUBLIC
#        ${THIS_PACKAGE_INCLUDE_DEPENDS}
#)

target_link_libraries(cmexa_base PUBLIC ${cmeresearch_msgs_TARGETS}
        hardware_interface::hardware_interface
        hardware_interface::mock_components
        pluginlib::pluginlib
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
)

# Export hardware plugins
#pluginlib_export_plugin_description_file(hardware_interface ros2_control_cmexa.xml)
pluginlib_export_plugin_description_file(hardware_interface ros2_control_cmex.xml)


install(
        DIRECTORY include/
        DESTINATION include/cmexa_base
)

install(TARGETS cmexa_base
        EXPORT export_cmexa_base
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

ament_export_targets(export_cmexa_base HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
