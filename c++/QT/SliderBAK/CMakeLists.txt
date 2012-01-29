
cmake_minimum_required(VERSION 2.6)

PROJECT(ProgressBar)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(UISrcs main.ui)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(ProgressBar ProgressBar.cpp ${UISrcs})
TARGET_LINK_LIBRARIES(ProgressBar ${QT_LIBRARIES})
