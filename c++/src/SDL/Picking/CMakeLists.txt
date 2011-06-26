cmake_minimum_required(VERSION 2.6)

PROJECT(SDLPicking)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp") #g++

INCLUDE_DIRECTORIES(/usr/include/)

LINK_DIRECTORIES(/usr/lib)

ADD_EXECUTABLE(SDLPick SDLPick.cpp
SDL_Helper.h SDL_Helper.cpp)
TARGET_LINK_LIBRARIES(SDLPick glut GLU GL Xmu X11 SDL SDL_image)
