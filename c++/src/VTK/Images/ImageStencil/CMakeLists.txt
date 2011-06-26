cmake_minimum_required(VERSION 2.6)

PROJECT(ImageStencil)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageStencil ImageStencil.cxx)
TARGET_LINK_LIBRARIES(ImageStencil vtkHybrid)
