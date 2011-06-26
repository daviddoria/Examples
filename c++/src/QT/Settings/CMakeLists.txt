cmake_minimum_required(VERSION 2.6)

PROJECT(Settings)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

INCLUDE_DIRECTORIES(${QT_QTXML_INCLUDE_DIR}) # Prevents "QtXml: No such file or directory"

ADD_EXECUTABLE(Settings Settings.cpp)
message(${QT_LIBRARIES})
TARGET_LINK_LIBRARIES(Settings ${QT_LIBRARIES})
