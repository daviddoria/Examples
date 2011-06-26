cmake_minimum_required(VERSION 2.6)

PROJECT(LandmarkTransform)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(LandmarkTransform LandmarkTransform.cxx)
TARGET_LINK_LIBRARIES(LandmarkTransform vtkHybrid)

ADD_EXECUTABLE(Modes Modes.cxx)
TARGET_LINK_LIBRARIES(Modes vtkHybrid)
