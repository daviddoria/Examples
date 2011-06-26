cmake_minimum_required(VERSION 2.6)

PROJECT(ImageRegionExclusionIteratorWithIndex)

include_directories(/home/doriad/src/ITK/Wrapping/WrapITK/ExternalProjects/ItkVtkGlue/src/)
 
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ImageRegionExclusionIteratorWithIndex ImageRegionExclusionIteratorWithIndex.cxx)
TARGET_LINK_LIBRARIES(ImageRegionExclusionIteratorWithIndex vtkHybrid
ITKNumerics ITKBasicFilters ITKCommon ITKIO)
