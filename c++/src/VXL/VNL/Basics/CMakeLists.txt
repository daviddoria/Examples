cmake_minimum_required(VERSION 2.6)

Project(VNLBasics)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

ADD_EXECUTABLE(VNLBasics VNLBasics.cpp)
TARGET_LINK_LIBRARIES(VNLBasics vnl vgl)
