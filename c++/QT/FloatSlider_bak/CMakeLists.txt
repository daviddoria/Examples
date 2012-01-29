cmake_minimum_required(VERSION 2.6)

PROJECT(FloatSlider)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(UISrcs FloatSlider.ui)
QT4_WRAP_CPP(MOCSrcs FloatSlider.h)

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(FloatSlider main.cpp FloatSlider.cpp ${MOCSrcs} ${UISrcs})
TARGET_LINK_LIBRARIES(FloatSlider ${QT_LIBRARIES})

