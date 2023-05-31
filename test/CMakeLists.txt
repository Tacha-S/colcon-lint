cmake_minimum_required(VERSION 3.8)
project(test_project)

if(CMAKE_COMPILER_IS_GNUCXX
   OR CMAKE_CXX_COMPILER_ID
      MATCHES
      "Clang"
)
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

set(DEPENDENCIES
    geometry_msgs
    rclcpp
    std_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()

set(DEPS rclcpp std_msgs)

ament_export_dependencies(${DEPS})

ament_package()
