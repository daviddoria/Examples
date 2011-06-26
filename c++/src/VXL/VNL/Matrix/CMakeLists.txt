cmake_minimum_required(VERSION 2.6)

Project(Matrix)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

ADD_EXECUTABLE(Matrix Matrix.cpp)
TARGET_LINK_LIBRARIES(Matrix vnl)
