cmake_minimum_required(VERSION 2.6)

PROJECT(Connect)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_CPP(MOCSrcs MyWidget.h)

ADD_EXECUTABLE(Connect Connect.cpp MyWidget.cpp ${MOCSrcs})
TARGET_LINK_LIBRARIES(Connect ${QT_LIBRARIES})

