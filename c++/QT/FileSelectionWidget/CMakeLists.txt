cmake_minimum_required(VERSION 2.6)

PROJECT(FileSelectionWidget)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${QT_INCLUDE_DIR})

QT4_WRAP_UI(UISrcs form.ui FileSelectionWidget.ui)
QT4_WRAP_CPP(MOCSrcs form.h FileSelectionWidget.h)

ADD_EXECUTABLE(FileSelectionWidget main.cpp form.cpp FileSelectionWidget.cpp ${MOCSrcs} ${UISrcs})
TARGET_LINK_LIBRARIES(FileSelectionWidget ${QT_LIBRARIES})

