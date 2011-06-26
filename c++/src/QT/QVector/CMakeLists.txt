cmake_minimum_required(VERSION 2.6)

PROJECT(QVector)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(QVector QVector.cpp)
TARGET_LINK_LIBRARIES(QVector ${QT_LIBRARIES})

