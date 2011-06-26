cmake_minimum_required(VERSION 2.6)

PROJECT(Signal)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_CPP(MOCSrcs MyWidget.h)

ADD_EXECUTABLE(Signal Signal.cpp MyWidget.cpp ${MOCSrcs})
TARGET_LINK_LIBRARIES(Signal ${QT_LIBRARIES})

