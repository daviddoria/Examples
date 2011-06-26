cmake_minimum_required(VERSION 2.6)

Project(rkpl)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

ADD_EXECUTABLE(rkpl rkpl.cpp)


TARGET_LINK_LIBRARIES(rkpl 
vnl vgl vcl rrel
)
