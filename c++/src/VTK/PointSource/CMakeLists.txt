cmake_minimum_required(VERSION 2.6)

PROJECT(PointSource)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PointSource PointSource.cxx)
TARGET_LINK_LIBRARIES(PointSource vtkHybrid)
