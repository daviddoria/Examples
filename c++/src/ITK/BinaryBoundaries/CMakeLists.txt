cmake_minimum_required(VERSION 2.6)

PROJECT(BinaryBoundaries)

include_directories(/home/doriad/src/ITK/Wrapping/WrapITK/ExternalProjects/ItkVtkGlue/src/)
include_directories(/home/doriad/ITKWikiExamples/ItkVtkGlue)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(BinaryBoundaries BinaryBoundaries.cxx /home/doriad/ITKWikiExamples/ItkVtkGlue/QuickView.cxx)
TARGET_LINK_LIBRARIES(BinaryBoundaries
vtkHybrid
ITKBasicFilters ITKCommon ITKIO)