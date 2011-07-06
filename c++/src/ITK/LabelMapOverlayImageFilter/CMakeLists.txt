cmake_minimum_required(VERSION 2.6)

PROJECT(LabelMapOverlayImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(LabelMapOverlayImageFilter LabelMapOverlayImageFilter.cxx)
TARGET_LINK_LIBRARIES(LabelMapOverlayImageFilter ${ITK_LIBRARIES})
