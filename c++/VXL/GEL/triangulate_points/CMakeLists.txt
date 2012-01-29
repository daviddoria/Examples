cmake_minimum_required(VERSION 2.6)

Project(triangulate_points)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)
INCLUDE_DIRECTORIES(${VXL_GEL_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${VXL_GEL_INCLUDE_DIR}/mrc)

ADD_EXECUTABLE(triangulate_points triangulate_points.cpp)
TARGET_LINK_LIBRARIES(triangulate_points vpgl vpgl_algo)
