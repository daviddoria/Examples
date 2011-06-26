cmake_minimum_required(VERSION 2.6)
PROJECT(ImageActor)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageActor ImageActor.cxx)
TARGET_LINK_LIBRARIES(ImageActor vtkHybrid)
