cmake_minimum_required(VERSION 2.6)

PROJECT(RandomGraphSource)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(RandomGraphSource RandomGraphSource.cxx)
TARGET_LINK_LIBRARIES(RandomGraphSource vtkHybrid  vtkViews)

