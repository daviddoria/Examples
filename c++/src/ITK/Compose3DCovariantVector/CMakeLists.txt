cmake_minimum_required(VERSION 2.6)

PROJECT(Compose3DCovariantVector)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(Compose3DCovariantVector Compose3DCovariantVector.cxx)
TARGET_LINK_LIBRARIES(Compose3DCovariantVector ITKIO)
