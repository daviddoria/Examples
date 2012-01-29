cmake_minimum_required(VERSION 2.6)

PROJECT(ListView)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(ListView_UI form.ui)
QT4_WRAP_CPP(ListView_MOC form.h)

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(ListView main.cpp form.h form.cpp ${ListView_UI} ${ListView_MOC})
TARGET_LINK_LIBRARIES(ListView ${QT_LIBRARIES})
