cmake_minimum_required(VERSION 2.6)

PROJECT(Test)

ADD_EXECUTABLE(Test Test.cpp)

TARGET_LINK_LIBRARIES(Test
glut GLU GL Xmu X11
SDL SDL_image SDL_ttf
)
