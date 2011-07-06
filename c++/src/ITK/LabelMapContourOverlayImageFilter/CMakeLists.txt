cmake_minimum_required(VERSION 2.6)

PROJECT(LabelMapContourOverlayImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(LabelMapContourOverlayImageFilter LabelMapContourOverlayImageFilter.cxx)
TARGET_LINK_LIBRARIES(LabelMapContourOverlayImageFilter ${ITK_LIBRARIES})
