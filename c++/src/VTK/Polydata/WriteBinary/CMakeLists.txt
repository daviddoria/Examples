cmake_minimum_required(VERSION 2.6)

PROJECT(WriteBinary)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(WriteBinary WriteBinary.cxx)
TARGET_LINK_LIBRARIES(WriteBinary vtkHybrid)
