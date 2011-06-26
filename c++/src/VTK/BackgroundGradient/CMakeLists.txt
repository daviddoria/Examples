cmake_minimum_required(VERSION 2.6)
PROJECT(BackgroundGradient)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(BackgroundGradient BackgroundGradient.cxx)
TARGET_LINK_LIBRARIES(BackgroundGradient vtkHybrid)

