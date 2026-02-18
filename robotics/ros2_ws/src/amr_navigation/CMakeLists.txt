cmake_minimum_required(VERSION 3.8)
project(amr_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
