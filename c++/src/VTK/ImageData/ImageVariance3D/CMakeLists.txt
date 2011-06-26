cmake_minimum_required(VERSION 2.6)
PROJECT(ImageVariance3D)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageVariance3D ImageVariance3D.cxx)
TARGET_LINK_LIBRARIES(ImageVariance3D vtkHybrid)

