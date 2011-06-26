cmake_minimum_required(VERSION 2.6)

PROJECT(InteractWithImage)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(InteractWithImage InteractWithImage.cxx)
TARGET_LINK_LIBRARIES(InteractWithImage vtkHybrid)


