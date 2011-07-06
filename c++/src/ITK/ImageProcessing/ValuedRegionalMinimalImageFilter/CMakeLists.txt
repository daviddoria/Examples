cmake_minimum_required(VERSION 2.6)

PROJECT(ValuedRegionalMaximalImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ValuedRegionalMaximalImageFilter ValuedRegionalMaximalImageFilter.cxx)
TARGET_LINK_LIBRARIES(ValuedRegionalMaximalImageFilter ${ITK_LIBRARIES})