cmake_minimum_required(VERSION 2.6)

PROJECT(ConnectedComponents)

include_directories(/home/doriad/src/ITK/Wrapping/WrapITK/ExternalProjects/ItkVtkGlue/src/)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ConnectedComponents ConnectedComponents.cpp)
TARGET_LINK_LIBRARIES(ConnectedComponents
vtkHybrid
ITKBasicFilters ITKCommon ITKIO)

