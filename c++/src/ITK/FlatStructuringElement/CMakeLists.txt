cmake_minimum_required(VERSION 2.6)

PROJECT(FlatStructuringElement)

include_directories(/home/doriad/src/ITK/Wrapping/WrapITK/ExternalProjects/ItkVtkGlue/src/)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(FlatStructuringElement FlatStructuringElement.cxx)
TARGET_LINK_LIBRARIES(FlatStructuringElement
vtkHybrid
ITKBasicFilters ITKCommon ITKIO)
