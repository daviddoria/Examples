cmake_minimum_required(VERSION 2.6)

Project(VCSLCoordinateSystems)

FIND_PACKAGE(VXL REQUIRED)

INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

ADD_EXECUTABLE(VCSLCoordinateSystems VCSLCoordinateSystems.cpp)

TARGET_LINK_LIBRARIES(VCSLCoordinateSystems 
vnl 
vgl
vcsl
rrel)
