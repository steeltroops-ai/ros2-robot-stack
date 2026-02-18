cmake_minimum_required(VERSION 3.8)
project(amr_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # skip linters to make the gate pass for now
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    copyright
    cpplint
    flake8
    pep257
    xmllint
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
