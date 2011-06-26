cmake_minimum_required(VERSION 2.6)

PROJECT(ImagePixelOrder)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ImagePixelOrder ImagePixelOrder.cpp)
TARGET_LINK_LIBRARIES(ImagePixelOrder ITKIO)
