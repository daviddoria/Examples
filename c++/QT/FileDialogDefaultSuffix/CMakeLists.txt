cmake_minimum_required(VERSION 2.6)

PROJECT(FileDialogDefaultSuffix)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

QT4_WRAP_UI(UISrcs form.ui)
QT4_WRAP_CPP(MOCSrcs form.h)

ADD_EXECUTABLE(FileDialogDefaultSuffix FileDialogDefaultSuffix.cpp form.cpp
${MOCSrcs} ${UISrcs})
TARGET_LINK_LIBRARIES(FileDialogDefaultSuffix ${QT_LIBRARIES})
