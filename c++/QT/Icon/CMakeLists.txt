cmake_minimum_required(VERSION 2.6)

PROJECT(Icon)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(Toolbar_UI form.ui)
QT4_WRAP_CPP(Toolbar_MOC form.h)

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(Icon Icon.cpp form.h form.cpp ${Toolbar_UI} ${Toolbar_MOC})
TARGET_LINK_LIBRARIES(Icon ${QT_LIBRARIES})
