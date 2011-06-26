cmake_minimum_required(VERSION 2.6)

PROJECT(BlobSpatialObject)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(BlobSpatialObject BlobSpatialObject.cxx)
TARGET_LINK_LIBRARIES(BlobSpatialObject ITKIO)
