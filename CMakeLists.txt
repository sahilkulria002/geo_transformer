cmake_minimum_required(VERSION 3.8)
project(geo_transformer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Find dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rosidl_default_generators REQUIRED)

pkg_check_modules(GEOGRAPHICLIB REQUIRED geographiclib)

# --- Generate custom services ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetOrigin.srv"
)

ament_export_dependencies(rosidl_default_runtime)

# --- Executables ---
add_executable(set_origin_node src/set_origin_node.cpp)

target_include_directories(set_origin_node PUBLIC
  ${GEOGRAPHICLIB_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(set_origin_node
  rclcpp
  std_msgs
)

target_link_libraries(set_origin_node ${GEOGRAPHICLIB_LIBRARIES})

rosidl_target_interfaces(set_origin_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS set_origin_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Testing (optional) ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
