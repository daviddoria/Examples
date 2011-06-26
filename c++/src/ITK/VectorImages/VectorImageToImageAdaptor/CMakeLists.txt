cmake_minimum_required(VERSION 2.6)

PROJECT(VectorImageToImageAdaptor)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(VectorImageToImageAdaptor VectorImageToImageAdaptor.cxx)
TARGET_LINK_LIBRARIES(VectorImageToImageAdaptor ${ITK_LIBRARIES})
