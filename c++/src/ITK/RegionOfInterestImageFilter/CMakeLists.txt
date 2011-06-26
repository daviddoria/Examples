cmake_minimum_required(VERSION 2.6)

PROJECT(RegionOfInterestImageFilter)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

#include_directories(/home/doriad/src/ITK/Wrapping/WrapITK/ExternalProjects/ItkVtkGlue/src/)
include_directories(${ITK_SOURCE_DIR}/Wrapping/WrapITK/ExternalProjects/ItkVtkGlue/src/)

ADD_EXECUTABLE(RegionOfInterestImageFilter RegionOfInterestImageFilter.cpp)
TARGET_LINK_LIBRARIES(RegionOfInterestImageFilter
vtkHybrid
ITKBasicFilters ITKCommon ITKIO)
