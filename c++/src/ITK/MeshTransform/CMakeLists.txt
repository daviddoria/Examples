cmake_minimum_required(VERSION 2.6)

PROJECT(Transform)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

##################
ADD_EXECUTABLE(Transform Transform.cpp)
TARGET_LINK_LIBRARIES(Transform ITKIO)

