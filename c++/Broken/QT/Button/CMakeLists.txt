cmake_minimum_required(VERSION 2.6)

PROJECT(Button)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

QT4_WRAP_UI(UISrcs main.ui)
QT4_WRAP_CPP(MOCSrcs mypush.h)

ADD_EXECUTABLE(Button Button.cpp mypush.cpp ${UISrcs})
TARGET_LINK_LIBRARIES(Button ${QT_LIBRARIES})

