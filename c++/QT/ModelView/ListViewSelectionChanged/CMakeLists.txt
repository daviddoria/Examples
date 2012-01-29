cmake_minimum_required(VERSION 2.6)

PROJECT(ListViewSelectionChanged)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(ListViewSelectionChanged_UI form.ui)
QT4_WRAP_CPP(ListViewSelectionChanged_MOC form.h)

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(ListViewSelectionChanged main.cpp form.h form.cpp ${ListViewSelectionChanged_UI} ${ListViewSelectionChanged_MOC})
TARGET_LINK_LIBRARIES(ListViewSelectionChanged ${QT_LIBRARIES})
