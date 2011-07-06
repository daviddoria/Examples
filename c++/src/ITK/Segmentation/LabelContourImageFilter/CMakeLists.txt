cmake_minimum_required(VERSION 2.6)

PROJECT(LabelContourImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(LabelContourImageFilter LabelContourImageFilter.cxx)
TARGET_LINK_LIBRARIES(LabelContourImageFilter ${ITK_LIBRARIES})