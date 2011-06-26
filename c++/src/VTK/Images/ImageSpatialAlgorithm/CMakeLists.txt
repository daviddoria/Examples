cmake_minimum_required(VERSION 2.6)

PROJECT(ImageSpatialAlgorithm)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageSpatialAlgorithm ImageSpatialAlgorithm.cxx)
TARGET_LINK_LIBRARIES(ImageSpatialAlgorithm vtkHybrid)
