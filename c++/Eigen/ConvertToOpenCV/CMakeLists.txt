cmake_minimum_required(VERSION 2.6)

PROJECT(ConvertToOpenCV)

# must set EIGEN3_INCLUDE_DIR to ~/src/Eigen (or where ever Eigen lives)

#set(EIGEN3_INCLUDE_DIR ~/src/eigen)

#SET(CMAKE_MODULE_PATH /media/portable/src/Eigen/cmake)
#SET(CMAKE_MODULE_PATH .)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR} ${CMAKE_MODULE_PATH})
#set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})

FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

ADD_EXECUTABLE(ConvertToOpenCV ConvertToOpenCV.cpp )
