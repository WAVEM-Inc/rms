cmake_minimum_required(VERSION 3.8)
project(ktp_data_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  include/${PROJECT_NAME}
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ktp_data_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(SOURCES
  src/utils/utils.cxx
  src/builder/builder.cxx
  src/builder/request/request_builder.cxx
  src/builder/request/control/control_builder.cxx
  src/builder/request/mission/mission_builder.cxx
  src/builder/request/object_detect/object_detect_builder.cxx
  src/builder/response/response_builder.cxx
  src/builder/response/control_report/control_report_builder.cxx
  src/builder/response/error_report/error_report_builder.cxx
  src/builder/response/graph_list/graph_list_builder.cxx
  src/builder/response/robot_status/robot_status_builder.cxx
  src/builder/response/service_status/service_status_builder.cxx
  src/manager/manager.cxx
  src/manager/request/request_manager.cxx
  src/manager/request/control/control_manager.cxx
  src/manager/request/mission/mission_manager.cxx
  src/manager/request/object_detect/object_detect_manager.cxx
  src/manager/response/response_manager.cxx
  src/manager/response/control_report/control_report_manager.cxx
  src/manager/response/error_report/error_report_manager.cxx
  src/manager/response/graph_list/graph_list_manager.cxx
  src/manager/response/robot_status/robot_status_manager.cxx
  src/manager/response/service_status/service_status_manager.cxx
  src/main.cxx
)

set(EXEC_NAME
  ${PROJECT_NAME}
)

add_executable(${EXEC_NAME}
  ${SOURCES}
)

set(DEPENDS
  rclcpp
  ktp_data_msgs
  sensor_msgs
  geometry_msgs
)

ament_target_dependencies(${EXEC_NAME}
  ${DEPENDS}
)

install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
