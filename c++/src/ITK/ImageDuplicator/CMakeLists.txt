cmake_minimum_required(VERSION 2.6)

PROJECT(ImageDuplicator)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ImageDuplicator ImageDuplicator.cxx)
TARGET_LINK_LIBRARIES(ImageDuplicator ITKIO)
