cmake_minimum_required(VERSION 3.16)
project(imm_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  geometry_msgs
  kdl_parser
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  imm_controller_parameters
  src/imm_parameters.yaml
)


add_library(imm_controller SHARED
  src/imm_controller.cpp
)
target_compile_features(imm_controller PUBLIC cxx_std_17)
target_include_directories(imm_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/imm_controller>
)

target_link_libraries(imm_controller PUBLIC
  imm_controller_parameters
)

ament_target_dependencies(imm_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(imm_controller PRIVATE "IMM_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface imm_plugin.xml)


# if(BUILD_TESTING)
#   find_package(ament_cmake_gmock REQUIRED)
#   find_package(controller_manager REQUIRED)
#   find_package(ros2_control_test_assets REQUIRED)

#   ament_add_gmock(test_load_imm_controller
#     test/test_load_imm_controller.cpp
#   )
#   target_link_libraries(test_load_imm_controller
#     imm_controller
#   )
#   ament_target_dependencies(test_load_imm_controller
#     controller_manager
#     hardware_interface
#     ros2_control_test_assets
#   )

#   ament_add_gmock(test_imm_controller
#     test/test_imm_controller.cpp
#   )
#   target_link_libraries(test_imm_controller
#     imm_controller
#   )

#   ament_add_gmock(test_load_multi_interface_imm_controller
#     test/test_load_multi_interface_imm_controller.cpp
#   )
#   target_link_libraries(test_load_multi_interface_imm_controller
#     imm_controller
#   )
#   ament_target_dependencies(test_load_multi_interface_imm_controller
#     controller_manager
#     hardware_interface
#     ros2_control_test_assets
#   )

#   ament_add_gmock(test_multi_interface_imm_controller
#     test/test_multi_interface_imm_controller.cpp
#   )
#   target_link_libraries(test_multi_interface_imm_controller
#     imm_controller
#   )
# endif()

install(
  DIRECTORY include/
  DESTINATION include/imm_controller
)

install(
  TARGETS
    imm_controller
    imm_controller_parameters
  EXPORT export_imm_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_imm_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
