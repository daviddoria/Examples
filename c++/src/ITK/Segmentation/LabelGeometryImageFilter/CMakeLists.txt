cmake_minimum_required(VERSION 2.6)

PROJECT(LabelGeometryImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(LabelGeometryImageFilter LabelGeometryImageFilter.cxx)
TARGET_LINK_LIBRARIES(LabelGeometryImageFilter ${ITK_LIBRARIES})
