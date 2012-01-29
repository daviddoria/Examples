cmake_minimum_required(VERSION 2.6)

PROJECT(OpenGLNoWindow)


FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp") #g++


ADD_EXECUTABLE(OpenGLNoWindow
OpenGLNoWindow.cpp
)

TARGET_LINK_LIBRARIES(OpenGLNoWindow
vil vnl vgl vgl_algo
glut GLU GL Xmu X11 
)
