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

set(SOURCES
  src/ktp_data_manager/ktp_data_manager.cxx
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
