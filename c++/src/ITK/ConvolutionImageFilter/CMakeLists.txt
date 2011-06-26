cmake_minimum_required(VERSION 2.6)

PROJECT(ConvolutionImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

# ADD_EXECUTABLE(ConvolutionImageFilterScalar ConvolutionImageFilterScalar.cxx)
# TARGET_LINK_LIBRARIES(ConvolutionImageFilterScalar
# ITKBasicFilters ITKIO ITKCommon)
# 
# ADD_EXECUTABLE(ConvolutionImageFilterVector ConvolutionImageFilterVector.cxx)
# TARGET_LINK_LIBRARIES(ConvolutionImageFilterVector
# ITKBasicFilters ITKIO ITKCommon)

ADD_EXECUTABLE(ConvolutionImageFilterDifferentKernel ConvolutionImageFilterDifferentKernel.cxx)
TARGET_LINK_LIBRARIES(ConvolutionImageFilterDifferentKernel
ITKBasicFilters ITKIO ITKCommon)
