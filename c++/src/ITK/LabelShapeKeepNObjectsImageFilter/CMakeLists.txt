cmake_minimum_required(VERSION 2.6)

PROJECT(LabelShapeKeepNObjectsImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(LabelShapeKeepNObjectsImageFilter LabelShapeKeepNObjectsImageFilter.cxx)
TARGET_LINK_LIBRARIES(LabelShapeKeepNObjectsImageFilter ${ITK_LIBRARIES} )
