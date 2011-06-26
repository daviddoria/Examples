cmake_minimum_required(VERSION 2.6)

PROJECT(WriteVTIFile)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(WriteVTIFile WriteVTIFile.cxx)
TARGET_LINK_LIBRARIES(WriteVTIFile vtkHybrid)

ADD_EXECUTABLE(WriteVTIFileVectors WriteVTIFileVectors.cxx)
TARGET_LINK_LIBRARIES(WriteVTIFileVectors vtkHybrid)
