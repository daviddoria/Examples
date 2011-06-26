cmake_minimum_required(VERSION 2.6)

PROJECT(Matrix)

# must set EIGEN3_INCLUDE_DIR to ~/src/Eigen (or where ever Eigen lives)

SET(CMAKE_MODULE_PATH /media/portable/src/Eigen/cmake)

FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

ADD_EXECUTABLE(Matrix Matrix.cpp )
