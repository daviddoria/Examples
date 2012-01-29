cmake_minimum_required(VERSION 2.6)

PROJECT(Property)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(UISrcs main.ui)
QT4_WRAP_CPP(MOCSrcs form.h)

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(Property Property.cpp form.h form.cpp ${UISrcs} ${MOCSrcs})
TARGET_LINK_LIBRARIES(Property ${QT_LIBRARIES})
