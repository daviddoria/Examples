cmake_minimum_required(VERSION 2.6)

PROJECT(Calculator)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(UISrcs calculatorform.ui)
QT4_WRAP_CPP(MOCSrcs calculatorform.h)

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(Calculator main.cpp calculatorform.h calculatorform.cpp ${MOCSrcs} ${UISrcs})
TARGET_LINK_LIBRARIES(Calculator ${QT_LIBRARIES})

