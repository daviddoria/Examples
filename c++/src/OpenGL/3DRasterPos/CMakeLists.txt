cmake_minimum_required(VERSION 2.6)

PROJECT(OpenGL3DRasterPos)


INCLUDE_DIRECTORIES(/usr/include/GL/)
			
LINK_DIRECTORIES(/usr/lib)

ADD_EXECUTABLE(RasterPos RasterPos.cpp	)

TARGET_LINK_LIBRARIES(RasterPos glut GLU GL Xmu X11)
