cmake_minimum_required(VERSION 2.6)

PROJECT(LineSpatialObject)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(LineSpatialObject LineSpatialObject.cxx)
TARGET_LINK_LIBRARIES(LineSpatialObject ITKIO)
