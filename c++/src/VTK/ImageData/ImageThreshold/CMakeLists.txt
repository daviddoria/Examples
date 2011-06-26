cmake_minimum_required(VERSION 2.6)
PROJECT(ImageThreshold)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageThreshold ImageThreshold.cxx)
TARGET_LINK_LIBRARIES(ImageThreshold vtkHybrid)

