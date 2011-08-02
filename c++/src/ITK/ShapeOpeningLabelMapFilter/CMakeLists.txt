cmake_minimum_required(VERSION 2.6)

PROJECT(ShapeOpeningLabelMapFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ShapeOpeningLabelMapFilter ShapeOpeningLabelMapFilter.cxx)
TARGET_LINK_LIBRARIES(ShapeOpeningLabelMapFilter ${ITK_LIBRARIES} )
