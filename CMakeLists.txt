cmake_minimum_required(VERSION 3.8...3.22)
project(tstar_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch src/tstar_description/meshes src/tstar_description/urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
