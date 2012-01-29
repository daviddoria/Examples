cmake_minimum_required(VERSION 2.6)

PROJECT(TreeView)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(TreeView_UI form.ui)
QT4_WRAP_CPP(TreeView_MOC form.h)

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(TreeView main.cpp form.h form.cpp ${TreeView_UI} ${TreeView_MOC})
TARGET_LINK_LIBRARIES(TreeView ${QT_LIBRARIES})
