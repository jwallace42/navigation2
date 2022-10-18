cmake_minimum_required(VERSION 3.8)
project(nav2_share_costmap)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_common REQUIRED)

nav2_package()

include_directories(
    include/${PROJECT_NAME}
)

set(executable_name share_costmap)

set(dependencies
    rclcpp
    nav2_planner
    nav2_controller
    nav2_behaviors
    nav2_common
    )

add_executable(${executable_name}
    src/main.cpp)

ament_target_dependencies(${executable_name}
    ${dependencies})

install(TARGETS ${executable_name}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    )

install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION include/${PROJECT_NAME}
    )

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
