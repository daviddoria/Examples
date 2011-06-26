cmake_minimum_required(VERSION 2.6)

PROJECT(EllipseSpatialObject)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(EllipseSpatialObject EllipseSpatialObject.cxx)
TARGET_LINK_LIBRARIES(EllipseSpatialObject ITKIO)
