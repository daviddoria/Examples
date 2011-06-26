cmake_minimum_required(VERSION 2.6)

Project(LineIntersection)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

ADD_EXECUTABLE(LineIntersection LineIntersection.cpp)

TARGET_LINK_LIBRARIES(LineIntersection vnl vnl_algo vgl vgl_algo)
