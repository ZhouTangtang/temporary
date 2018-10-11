cmake_minimum_required(VERSION 2.8)

project(jaco)
find_package(Eigen3)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(jaco test.cpp)