cmake_minimum_required(VERSION 2.6)

include_directories(/home/doriad/ITKWikiExamples/ItkVtkGlue)

PROJECT(ContourSpatialObject)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

ADD_EXECUTABLE(ContourSpatialObject ContourSpatialObject.cxx
/home/doriad/ITKWikiExamples/ItkVtkGlue/QuickView.cxx)
TARGET_LINK_LIBRARIES(ContourSpatialObject
ITKIO ITKBasicFilters ITKCommon
vtkHybrid)
