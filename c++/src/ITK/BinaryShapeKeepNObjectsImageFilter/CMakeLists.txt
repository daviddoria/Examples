cmake_minimum_required(VERSION 2.6)

PROJECT(BinaryShapeKeepNObjectsImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(BinaryShapeKeepNObjectsImageFilter
BinaryShapeKeepNObjectsImageFilter.cxx)
TARGET_LINK_LIBRARIES(BinaryShapeKeepNObjectsImageFilter
${ITK_LIBRARIES})
