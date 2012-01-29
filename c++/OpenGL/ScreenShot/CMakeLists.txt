cmake_minimum_required(VERSION 2.6)

PROJECT(Screenshot)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp") #g++

INCLUDE_DIRECTORIES(/usr/include/ /usr/local/include/)

LINK_DIRECTORIES(/usr/lib /usr/local/lib)

ADD_EXECUTABLE(Screenshot Test.cpp Screenshot.cpp)

TARGET_LINK_LIBRARIES(Screenshot
glut GLU GL Xmu X11
vil vgl vnl vgl_algo vbl
)
