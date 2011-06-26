cmake_minimum_required(VERSION 2.6)

PROJECT(KdTreeSelector)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(KdTreeSelector KdTreeSelector.cxx)
TARGET_LINK_LIBRARIES(KdTreeSelector vtkHybrid)
