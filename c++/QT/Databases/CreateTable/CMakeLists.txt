cmake_minimum_required(VERSION 2.6)

PROJECT(CreateTable)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(UISrcs CreateTable.ui)
QT4_WRAP_CPP(MOCSrcs form.h)

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
INCLUDE_DIRECTORIES(${QT_QTSQL_INCLUDE_DIR})

ADD_EXECUTABLE(CreateTable CreateTable.cpp form.h form.cpp ${MOCSrcs} ${UISrcs})
TARGET_LINK_LIBRARIES(CreateTable ${QT_LIBRARIES}  ${QT_QTSQL_LIBRARY_RELEASE})

