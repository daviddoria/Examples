cmake_minimum_required(VERSION 2.6)
PROJECT(SceneBounds)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SceneBounds SceneBounds.cxx)
TARGET_LINK_LIBRARIES(SceneBounds vtkHybrid)

