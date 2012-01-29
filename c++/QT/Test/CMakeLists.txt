cmake_minimum_required(VERSION 2.6)

PROJECT(Test)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

#QT4_WRAP_UI(UISrcs Test.ui)
#QT4_WRAP_CPP(MOCSrcs Test.h)

#include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

#ADD_EXECUTABLE(Test Test.cpp Test.h ${MOCSrcs} ${UISrcs})
ADD_EXECUTABLE(Test Test.cpp)
TARGET_LINK_LIBRARIES(Test ${QT_LIBRARIES})

