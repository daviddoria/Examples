cmake_minimum_required(VERSION 2.6)

PROJECT(SDLThreads)

ADD_EXECUTABLE(SDLThreads SDLThreads.cpp)

TARGET_LINK_LIBRARIES(SDLThreads glut GLU GL Xmu X11 SDL SDL_image)
