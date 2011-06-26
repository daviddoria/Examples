cmake_minimum_required(VERSION 2.6)

Project(VGLBasics)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

ADD_EXECUTABLE(VGLBasics VGLBasics.cpp)

TARGET_LINK_LIBRARIES(VGLBasics vnl vnl_algo vgl vgl_algo)
