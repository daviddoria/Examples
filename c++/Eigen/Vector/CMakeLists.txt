cmake_minimum_required(VERSION 2.6)

PROJECT(Vector)
include_directories(/media/portable/src/Eigen)
# FIND_PACKAGE(Eigen3 REQUIRED)
# INCLUDE(${Eigen_USE_FILE})

ADD_EXECUTABLE(Vector Vector.cpp )
