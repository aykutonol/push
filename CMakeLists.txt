cmake_minimum_required(VERSION 2.8.3)
project(push)

add_compile_options(-std=c+11)

find_package(catkin   REQUIRED)
find_package(Eigen3   REQUIRED)
find_package(yaml-cpp REQUIRED)

set(MUJOCO_DIR "$ENV{MJ_HOME}")

catkin_package()

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${MUJOCO_DIR}/include
)
