cmake_minimum_required(VERSION 2.6)

Project(VBLBasics)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

ADD_EXECUTABLE(VBLBasics VBLBasics.cpp)


TARGET_LINK_LIBRARIES(VBLBasics vbl )
