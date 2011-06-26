cmake_minimum_required(VERSION 2.6)

PROJECT(ImageImport)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ImageImport ImageImport.cxx)
TARGET_LINK_LIBRARIES(ImageImport vtkHybrid)

