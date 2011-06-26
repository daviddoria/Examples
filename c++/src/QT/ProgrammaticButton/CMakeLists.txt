cmake_minimum_required(VERSION 2.6)

PROJECT(ProgrammaticButton)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

ADD_EXECUTABLE(ProgrammaticButton ProgrammaticButton.cpp)
TARGET_LINK_LIBRARIES(ProgrammaticButton ${QT_LIBRARIES})
