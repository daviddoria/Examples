cmake_minimum_required(VERSION 2.6)

PROJECT(SignalArray)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_CPP(MOCSrcs MyWidget.h SmallWidget.h)

ADD_EXECUTABLE(SignalArray SignalArray.cpp MyWidget.cpp SmallWidget.cpp ${MOCSrcs})
TARGET_LINK_LIBRARIES(SignalArray ${QT_LIBRARIES})

