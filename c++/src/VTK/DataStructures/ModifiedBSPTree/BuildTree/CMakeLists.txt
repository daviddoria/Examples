cmake_minimum_required(VERSION 2.6)

PROJECT(BuildTree)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(BuildTree BuildTree.cxx)
TARGET_LINK_LIBRARIES(BuildTree vtkHybrid)
