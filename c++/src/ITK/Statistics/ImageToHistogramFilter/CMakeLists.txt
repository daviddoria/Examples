cmake_minimum_required(VERSION 2.6)

PROJECT(ImageToHistogramFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ImageToHistogramFilter ImageToHistogramFilter.cxx)
TARGET_LINK_LIBRARIES(ImageToHistogramFilter
ITKBasicFilters ITKCommon ITKIO ITKStatistics)

ADD_EXECUTABLE(SetBug SetBug.cxx)
TARGET_LINK_LIBRARIES(SetBug
ITKBasicFilters ITKCommon ITKIO ITKStatistics)

ADD_EXECUTABLE(SetObjectBug SetObjectBug.cxx)
TARGET_LINK_LIBRARIES(SetObjectBug
ITKBasicFilters ITKCommon ITKIO ITKStatistics)

ADD_EXECUTABLE(ZeroComponentBug ZeroComponentBug.cxx)
TARGET_LINK_LIBRARIES(ZeroComponentBug
ITKBasicFilters ITKCommon ITKIO ITKStatistics)

ADD_EXECUTABLE(Test Test.cxx)
TARGET_LINK_LIBRARIES(Test
ITKBasicFilters ITKCommon ITKIO ITKStatistics)
