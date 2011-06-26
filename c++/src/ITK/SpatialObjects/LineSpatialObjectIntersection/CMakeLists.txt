cmake_minimum_required(VERSION 2.6)

PROJECT(LineSpatialObjectIntersection)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(LineSpatialObjectIntersection LineSpatialObjectIntersection.cxx)
TARGET_LINK_LIBRARIES(LineSpatialObjectIntersection ITKIO
vgl vgl_algo)
