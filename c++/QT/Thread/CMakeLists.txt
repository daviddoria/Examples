cmake_minimum_required(VERSION 2.6)

PROJECT(Thread)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_CPP(MOCSrcs MyClass.h form.h)
QT4_WRAP_UI(UISrcs form.ui)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(Thread Thread.cpp form.cpp ${MOCSrcs} ${UISrcs})
TARGET_LINK_LIBRARIES(Thread ${QT_LIBRARIES})

