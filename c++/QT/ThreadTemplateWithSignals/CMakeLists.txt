cmake_minimum_required(VERSION 2.6)

PROJECT(ThreadTemplateWithSignals)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

QT4_WRAP_CPP(MOCSrcs ThreadTemplateWithSignals.h)

ADD_EXECUTABLE(ThreadTemplateWithSignals Example.cpp ${MOCSrcs})
TARGET_LINK_LIBRARIES(ThreadTemplateWithSignals ${QT_LIBRARIES})

