cmake_minimum_required(VERSION 2.6)

PROJECT(LabelMapToLabelImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(LabelMapToLabelImageFilter LabelMapToLabelImageFilter.cxx)
TARGET_LINK_LIBRARIES(LabelMapToLabelImageFilter ${ITK_LIBRARIES})
