cmake_minimum_required(VERSION 2.6)

PROJECT(ClientData)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ClientData ClientData.cxx)
TARGET_LINK_LIBRARIES(ClientData vtkHybrid)
