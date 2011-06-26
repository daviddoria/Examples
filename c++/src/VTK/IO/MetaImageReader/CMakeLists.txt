cmake_minimum_required(VERSION 2.6)

PROJECT(MetaImageReader)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(MetaImageReader MetaImageReader.cxx)
TARGET_LINK_LIBRARIES(MetaImageReader vtkHybrid)

