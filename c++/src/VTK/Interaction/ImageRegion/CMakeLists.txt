cmake_minimum_required(VERSION 2.6)

PROJECT(ImageRegion)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageRegion ImageRegion.cxx)
TARGET_LINK_LIBRARIES(ImageRegion vtkHybrid vtkWidgets)
