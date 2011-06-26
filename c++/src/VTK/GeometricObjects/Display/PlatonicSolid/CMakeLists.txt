cmake_minimum_required(VERSION 2.6)

PROJECT(PlatonicSolid)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PlatonicSolid PlatonicSolid.cxx)

TARGET_LINK_LIBRARIES(PlatonicSolid vtkHybrid)
