cmake_minimum_required(VERSION 2.6)

PROJECT(SimplePointsWriter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SimplePointsWriter SimplePointsWriter.cxx)
TARGET_LINK_LIBRARIES(SimplePointsWriter vtkHybrid)

