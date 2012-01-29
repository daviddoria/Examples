cmake_minimum_required(VERSION 2.6)

PROJECT(ScopedPointer)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

ADD_EXECUTABLE(ScopedPointer ScopedPointer.cpp )
TARGET_LINK_LIBRARIES(ScopedPointer ${QT_LIBRARIES})
