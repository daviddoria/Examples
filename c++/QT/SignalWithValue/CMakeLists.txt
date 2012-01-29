cmake_minimum_required(VERSION 2.6)

PROJECT(SignalWithValue)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_CPP(MOCSrcs MyWidget.h)

ADD_EXECUTABLE(SignalWithValue SignalWithValue.cpp MyWidget.cpp ${MOCSrcs})
TARGET_LINK_LIBRARIES(SignalWithValue ${QT_LIBRARIES})

