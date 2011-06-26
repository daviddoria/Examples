cmake_minimum_required(VERSION 2.6)
PROJECT(ImageContinuousDilate3D)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageContinuousDilate3D ImageContinuousDilate3D.cxx)
TARGET_LINK_LIBRARIES(ImageContinuousDilate3D vtkHybrid)

