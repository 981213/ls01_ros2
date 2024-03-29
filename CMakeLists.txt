cmake_minimum_required(VERSION 3.8)
project(ls01)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
        src/ls01.cpp src/ls01b.cpp src/ls01d.cpp src/lsn10.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_components std_msgs sensor_msgs)
rclcpp_components_register_nodes(${PROJECT_NAME} "LS01::LS01B" "LS01::LS01D" "LS01::LSN10" "LS01::LSN10P")

add_executable(ls01b src/exe/ls01b.cpp)
target_link_libraries(ls01b ${PROJECT_NAME})
ament_target_dependencies(ls01b rclcpp)

add_executable(ls01d src/exe/ls01d.cpp)
target_link_libraries(ls01d ${PROJECT_NAME})
ament_target_dependencies(ls01d rclcpp)

add_executable(lsn10 src/exe/lsn10.cpp)
target_link_libraries(lsn10 ${PROJECT_NAME})
ament_target_dependencies(lsn10 rclcpp)

add_executable(lsn10p src/exe/lsn10p.cpp)
target_link_libraries(lsn10p ${PROJECT_NAME})
ament_target_dependencies(lsn10p rclcpp)

install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS
        ls01b
        ls01d
        lsn10
        lsn10p
        DESTINATION lib/${PROJECT_NAME})

install(
        DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
