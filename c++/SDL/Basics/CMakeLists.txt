cmake_minimum_required(VERSION 2.6)

PROJECT(SDLTest)

// http://sdl-draw.sourceforge.net/
FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp") #g++

INCLUDE_DIRECTORIES(/usr/include/)

LINK_DIRECTORIES(/usr/lib)

ADD_EXECUTABLE(SDLTest SDLTest.cpp
SDL_Helper.h SDL_Helper.cpp)

TARGET_LINK_LIBRARIES(SDLTest
glut GLU GL Xmu X11
SDL SDL_draw 
vnl
)
