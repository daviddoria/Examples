cmake_minimum_required(VERSION 2.6)
PROJECT(ImageShiftScale)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageShiftScale ImageShiftScale.cxx)
TARGET_LINK_LIBRARIES(ImageShiftScale vtkHybrid)

