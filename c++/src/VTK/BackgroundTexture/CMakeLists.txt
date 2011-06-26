cmake_minimum_required(VERSION 2.6)
PROJECT(BackgroundTexture)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(BackgroundTexture BackgroundTexture.cxx)
TARGET_LINK_LIBRARIES(BackgroundTexture vtkHybrid)

