cmake_minimum_required(VERSION 2.6)

PROJECT(DrawLine)

ADD_EXECUTABLE(DrawLine DrawLine.cpp)

TARGET_LINK_LIBRARIES(DrawLine
glut GLU GL Xmu X11
SDL SDL_image
)
