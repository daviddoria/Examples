cmake_minimum_required(VERSION 2.6)

PROJECT(Thread)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(Thread Thread.cpp)# ${MOCSrcs} ${UISrcs})
TARGET_LINK_LIBRARIES(Thread ${QT_LIBRARIES})

