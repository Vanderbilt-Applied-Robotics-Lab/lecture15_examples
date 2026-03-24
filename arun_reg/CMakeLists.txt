cmake_minimum_required(VERSION 3.8)
project(arun_reg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

## INCLUDE(S)
include_directories(include)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

# executables
add_executable(arun_reg src/arun_reg.cpp)

# dependencies
ament_target_dependencies(arun_reg rclcpp Eigen3)

target_link_libraries(arun_reg Eigen3::Eigen)


install(TARGETS arun_reg
  DESTINATION lib/${PROJECT_NAME})


ament_package()
