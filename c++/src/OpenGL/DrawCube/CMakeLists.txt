cmake_minimum_required(VERSION 2.6)

PROJECT(DrawCube)

find_package(GLUT)
find_package(OpenGL)

ADD_EXECUTABLE(DrawCube DrawCube.cpp)

TARGET_LINK_LIBRARIES(DrawCube ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
