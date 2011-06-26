cmake_minimum_required(VERSION 2.6)

Project(Test)

FIND_PACKAGE(VXL REQUIRED)

INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

ADD_EXECUTABLE(VULParse VULParse.cpp)

TARGET_LINK_LIBRARIES(VULParse vul rrel)
