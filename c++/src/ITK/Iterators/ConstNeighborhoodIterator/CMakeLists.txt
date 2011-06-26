cmake_minimum_required(VERSION 2.6)

PROJECT(ConstNeighborhoodIterator)

include_directories(/home/doriad/src/ITK/Wrapping/WrapITK/ExternalProjects/ItkVtkGlue/src/)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ConstNeighborhoodIterator ConstNeighborhoodIterator.cxx)
TARGET_LINK_LIBRARIES(ConstNeighborhoodIterator vtkHybrid ${ITK_LIBRARIES})
