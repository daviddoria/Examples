cmake_minimum_required(VERSION 2.6)

PROJECT(SingleScreenshot)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SingleScreenshot SingleScreenshot.cxx)
TARGET_LINK_LIBRARIES(SingleScreenshot vtkHybrid)

