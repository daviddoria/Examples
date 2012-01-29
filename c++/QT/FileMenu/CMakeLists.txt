cmake_minimum_required(VERSION 2.6)

PROJECT(FileMenu)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_UI(UISrcs form.ui)
QT4_WRAP_CPP(MOCSrcs filemenu.h)

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(FileMenu main.cpp filemenu.h filemenu.cpp ${MOCSrcs} ${UISrcs})
TARGET_LINK_LIBRARIES(FileMenu ${QT_LIBRARIES})

