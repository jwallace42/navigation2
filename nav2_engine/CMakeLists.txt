cmake_minimum_required(VERSION 3.8)
project(nav2_engine)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
    include
)

set(dependencies
    rclcpp
    nav2_util)

# Main library
add_library(${PROJECT_NAME}_server SHARED
    src/engine_server.cpp)

ament_target_dependencies(${PROJECT_NAME}_server
    ${dependencies})

# Muxer Library
add_library(${PROJECT_NAME}_muxer SHARED
    src/muxer.cpp
    src/engine_interface.cpp)

ament_target_dependencies(${PROJECT_NAME}_muxer
    ${dependencies})

# Engine Interface
#add_library(${PROJECT_NAME}_interface SHARED
#    src/engine_interface.cpp)
#
#ament_target_dependencies(${PROJECT_NAME}_interface
#    ${dependencies})



# Server node
add_executable(${PROJECT_NAME}_node
    src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
    ${dependencies})

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_server)

install(
    TARGETS ${PROJECT_NAME}_server
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION lib/${PROJECT_NAME})

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

ament_export_libraries(${PROJECT_NAME}_server)
ament_export_dependencies(${dependencies})
ament_package()
