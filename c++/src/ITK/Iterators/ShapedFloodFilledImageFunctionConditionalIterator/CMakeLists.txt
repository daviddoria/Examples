cmake_minimum_required(VERSION 2.6)

PROJECT(ShapedFloodFilledImageFunctionConditionalIterator)

include_directories(/home/doriad/ITKWikiExamples/ItkVtkGlue)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ShapedFloodFilledImageFunctionConditionalIterator ShapedFloodFilledImageFunctionConditionalIterator.cxx /home/doriad/ITKWikiExamples/ItkVtkGlue/QuickView.cxx)
TARGET_LINK_LIBRARIES(ShapedFloodFilledImageFunctionConditionalIterator
vtkHybrid
ITKIO ITKBasicFilters ITKCommon
)

