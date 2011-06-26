cmake_minimum_required(VERSION 2.6)

PROJECT(CreateAnother)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(CreateAnother CreateAnother.cpp)
TARGET_LINK_LIBRARIES(CreateAnother ${ITK_LIBRARIES})
