cmake_minimum_required(VERSION 2.6)

PROJECT(ScaleTransform)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ScaleTransform ScaleTransform.cxx)
TARGET_LINK_LIBRARIES(ScaleTransform ${ITK_LIBRARIES})

