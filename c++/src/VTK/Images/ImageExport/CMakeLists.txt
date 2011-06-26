cmake_minimum_required(VERSION 2.6)

PROJECT(ImageExport)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageExport ImageExport.cxx)
TARGET_LINK_LIBRARIES(ImageExport vtkHybrid)

