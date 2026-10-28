cmake_minimum_required(VERSION 3.8)
project(proxima_description)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(DIRECTORY
  launch
  meshes
  robots
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_xacro test/test_xacro.py)
endif()

ament_auto_package()
