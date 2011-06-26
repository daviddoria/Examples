cmake_minimum_required(VERSION 2.6)

PROJECT(BSplineScatteredDataPointSetToImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(BSplineScatteredDataPointSetToImageFilter BSplineScatteredDataPointSetToImageFilter.cxx)
TARGET_LINK_LIBRARIES(BSplineScatteredDataPointSetToImageFilter ${ITK_LIBRARIES})

ADD_EXECUTABLE(test test.cxx)
TARGET_LINK_LIBRARIES(test ${ITK_LIBRARIES})
