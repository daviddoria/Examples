cmake_minimum_required(VERSION 2.6)

PROJECT(SharedPointer)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

ADD_EXECUTABLE(SharedPointer SharedPointer.cpp)
TARGET_LINK_LIBRARIES(SharedPointer ${QT_LIBRARIES})
