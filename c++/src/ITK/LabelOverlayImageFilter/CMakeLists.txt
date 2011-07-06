cmake_minimum_required(VERSION 2.6)

PROJECT(LabelOverlayImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(LabelOverlayImageFilter LabelOverlayImageFilter.cxx)
TARGET_LINK_LIBRARIES(LabelOverlayImageFilter ${ITK_LIBRARIES})
