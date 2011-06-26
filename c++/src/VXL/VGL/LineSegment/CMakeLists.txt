cmake_minimum_required(VERSION 2.6)

Project(LineSegment)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

ADD_EXECUTABLE(LineSegment LineSegment.cpp)

TARGET_LINK_LIBRARIES(LineSegment vnl vnl_algo vgl vgl_algo)
