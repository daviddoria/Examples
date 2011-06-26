cmake_minimum_required(VERSION 2.6)

PROJECT(KeypressScreenshot)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(KeypressScreenshot KeypressScreenshot.cxx)
TARGET_LINK_LIBRARIES(KeypressScreenshot vtkHybrid)
