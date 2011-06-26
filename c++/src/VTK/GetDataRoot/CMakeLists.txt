cmake_minimum_required(VERSION 2.6)
PROJECT(GetDataRoot)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(GetDataRoot GetDataRoot.cxx)
TARGET_LINK_LIBRARIES(GetDataRoot vtkHybrid)

