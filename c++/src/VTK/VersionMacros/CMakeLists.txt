cmake_minimum_required(VERSION 2.6)

PROJECT(VersionMacros)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(VersionMacros VersionMacros.cxx)
TARGET_LINK_LIBRARIES(VersionMacros vtkHybrid)
