cmake_minimum_required(VERSION 2.6)

Project(Adjoint)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

ADD_EXECUTABLE(Adjoint Adjoint.cpp)
TARGET_LINK_LIBRARIES(Adjoint vnl vnl_algo )
