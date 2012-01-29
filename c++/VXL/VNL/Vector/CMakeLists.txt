cmake_minimum_required(VERSION 2.6)

Project(Vector)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

ADD_EXECUTABLE(Vector Vector.cpp)
TARGET_LINK_LIBRARIES(Vector vnl)
