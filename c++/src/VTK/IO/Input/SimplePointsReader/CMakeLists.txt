cmake_minimum_required(VERSION 2.6)

PROJECT(SimplePointsReader)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(SimplePointsReader SimplePointsReader.cpp)
TARGET_LINK_LIBRARIES(SimplePointsReader vtkHybrid)
