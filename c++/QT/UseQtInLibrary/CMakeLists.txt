cmake_minimum_required(VERSION 2.6)

PROJECT(UseQtInLibrary)

# FIND_PACKAGE(Qt4 REQUIRED)
# INCLUDE(${QT_USE_FILE})

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

add_library(mylib mylib.cpp)

FIND_PACKAGE(Qt4)
if(QT4_FOUND)
  message ( "QT4 found successfully." )
  INCLUDE(${QT_USE_FILE})
else ( QT4_FOUND )
  message ( "QT4 NOT found." )
endif ( QT4_FOUND )



ADD_EXECUTABLE(UseQtInLibrary main.cpp)
TARGET_LINK_LIBRARIES(UseQtInLibrary mylib ${QT_LIBRARIES})

