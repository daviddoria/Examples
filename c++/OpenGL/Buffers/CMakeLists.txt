cmake_minimum_required(VERSION 2.6)

PROJECT(OpenGLBuffers)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)


set(PROJECTS /media/portable/Projects/src)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp") #g++
INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} ${PROJECTS})
LINK_DIRECTORIES(/usr/lib)

ADD_EXECUTABLE(OpenGLBuffers OpenGLBuffers.cpp
${PROJECTS}/SDLHelpers/SDLHelpers.cpp
)
TARGET_LINK_LIBRARIES(OpenGLBuffers
glut GLU GL Xmu X11
SDL SDL_image
vnl
vgl
vil
)
