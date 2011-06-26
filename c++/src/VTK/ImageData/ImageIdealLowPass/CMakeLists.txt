cmake_minimum_required(VERSION 2.6)
PROJECT(ImageIdealLowPass)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageIdealLowPass ImageIdealLowPass.cxx)
TARGET_LINK_LIBRARIES(ImageIdealLowPass vtkHybrid)

