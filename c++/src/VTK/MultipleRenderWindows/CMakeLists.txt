cmake_minimum_required(VERSION 2.6)

PROJECT(MultipleRenderWindows)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(MultipleRenderWindows MultipleRenderWindows.cxx)

TARGET_LINK_LIBRARIES(MultipleRenderWindows vtkHybrid)
