cmake_minimum_required(VERSION 2.6)

PROJECT(SideBySideViewports)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SideBySideViewports SideBySideViewports.cxx)
TARGET_LINK_LIBRARIES(SideBySideViewports vtkHybrid)
