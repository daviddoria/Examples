cmake_minimum_required(VERSION 2.6)

PROJECT(Compose3DCovariantVectorImageFilter)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(Compose3DCovariantVectorImageFilter Compose3DCovariantVectorImageFilter.cxx)
TARGET_LINK_LIBRARIES(Compose3DCovariantVectorImageFilter ITKIO)
