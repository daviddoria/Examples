cmake_minimum_required(VERSION 2.6)
PROJECT(ImageContinuousErode3D)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageContinuousErode3D ImageContinuousErode3D.cxx)
TARGET_LINK_LIBRARIES(ImageContinuousErode3D vtkHybrid)

