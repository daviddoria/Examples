cmake_minimum_required(VERSION 2.6)

PROJECT(Transparency)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(Transparency Transparency.cxx)
TARGET_LINK_LIBRARIES(Transparency vtkHybrid)

