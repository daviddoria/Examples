cmake_minimum_required(VERSION 2.6)
PROJECT(ImageMedian3D)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageMedian3D ImageMedian3D.cxx)
TARGET_LINK_LIBRARIES(ImageMedian3D vtkHybrid)

