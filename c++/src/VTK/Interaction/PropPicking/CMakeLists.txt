cmake_minimum_required(VERSION 2.6)

PROJECT(PropPicking)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PropPicking PropPicking.cxx)
TARGET_LINK_LIBRARIES(PropPicking vtkHybrid)
