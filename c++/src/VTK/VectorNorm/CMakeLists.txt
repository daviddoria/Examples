cmake_minimum_required(VERSION 2.6)

PROJECT(VectorNorm)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(VectorNorm VectorNorm.cxx)
TARGET_LINK_LIBRARIES(VectorNorm vtkHybrid)
