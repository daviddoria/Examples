cmake_minimum_required(VERSION 2.6)

PROJECT(ImageRegionIterator)

include_directories(/home/doriad/src/ITK/Wrapping/WrapITK/ExternalProjects/ItkVtkGlue/src/)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ImageRegionIterator ImageRegionIterator.cxx)
TARGET_LINK_LIBRARIES(ImageRegionIterator vtkHybrid
ITKNumerics ITKBasicFilters ITKCommon ITKIO)

ADD_EXECUTABLE(RepeatIteration RepeatIteration.cxx)
TARGET_LINK_LIBRARIES(RepeatIteration ITKNumerics ITKBasicFilters ITKCommon ITKIO)

ADD_EXECUTABLE(VectorImageIterator VectorImageIterator.cxx)
TARGET_LINK_LIBRARIES(VectorImageIterator ITKNumerics ITKBasicFilters ITKCommon ITKIO)

