cmake_minimum_required(VERSION 2.6)

PROJECT(EasyRendering)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(EasyRendering Test.cpp EasyRenderer.cpp)

TARGET_LINK_LIBRARIES(EasyRendering vtkHybrid)
