cmake_minimum_required(VERSION 2.6)

PROJECT(OpenGL_ErrorChecking)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp") #g++

INCLUDE_DIRECTORIES(/usr/include/
/usr/local/include/
)

LINK_DIRECTORIES(/usr/lib
/usr/local/lib
)

ADD_EXECUTABLE(ErrorChecking ErrorChecking.cpp	  )

TARGET_LINK_LIBRARIES(ErrorChecking glut GLU GL Xmu X11 )
