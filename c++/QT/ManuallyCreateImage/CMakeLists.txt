cmake_minimum_required(VERSION 2.6)

PROJECT(ManuallyCreateImage)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(UISrcs form.ui)
QT4_WRAP_CPP(MOCSrcs form.h)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(ManuallyCreateImage main.cpp form.h form.cpp ${MOCSrcs} ${UISrcs})
TARGET_LINK_LIBRARIES(ManuallyCreateImage ${QT_LIBRARIES} ${ITK_LIBRARIES})

