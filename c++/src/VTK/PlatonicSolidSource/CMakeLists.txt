cmake_minimum_required(VERSION 2.6)
PROJECT(PlatonicSolidSource)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(PlatonicSolidSource PlatonicSolidSource.cxx)
TARGET_LINK_LIBRARIES(PlatonicSolidSource vtkHybrid)

