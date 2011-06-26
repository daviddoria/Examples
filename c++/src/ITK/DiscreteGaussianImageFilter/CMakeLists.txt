cmake_minimum_required(VERSION 2.6)

PROJECT(DiscreteGaussianImageFilter)

include_directories(/home/doriad/src/ITK/Wrapping/WrapITK/ExternalProjects/ItkVtkGlue/src/)

FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

# ADD_EXECUTABLE(DiscreteGaussianImageFilter DiscreteGaussianImageFilter.cxx)
# TARGET_LINK_LIBRARIES(DiscreteGaussianImageFilter
# vtkHybrid
# ITKBasicFilters ITKIO ITKCommon)
# 
# ADD_EXECUTABLE(DiscreteGaussianImageFilterColor DiscreteGaussianImageFilterColor.cxx)
# TARGET_LINK_LIBRARIES(DiscreteGaussianImageFilterColor
# ITKBasicFilters ITKIO ITKCommon)

ADD_EXECUTABLE(DiscreteGaussianImageFilterComponent DiscreteGaussianImageFilterComponent.cxx)
TARGET_LINK_LIBRARIES(DiscreteGaussianImageFilterComponent
ITKBasicFilters ITKIO ITKCommon)
