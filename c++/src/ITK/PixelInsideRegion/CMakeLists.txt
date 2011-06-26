cmake_minimum_required(VERSION 2.6)

PROJECT(PixelInsideRegion)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(PixelInsideRegion PixelInsideRegion.cxx)
TARGET_LINK_LIBRARIES(PixelInsideRegion ITKIO)