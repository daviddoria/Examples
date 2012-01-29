cmake_minimum_required(VERSION 2.6)

PROJECT(OpenGLTest)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp") #g++

set(PROJECTS /home/doriad/Projects)

set(GeometryDir ${PROJECTS}/GeometryLibrary/src)

set(FileFormatSources	${FileFormatsDir}/ObjFile.h		${FileFormatsDir}/ObjFile.cpp
			  ${FileFormatsDir}/VtkFile.h		${FileFormatsDir}/VtkFile.cpp
			  ${FileFormatsDir}/PtxFile.h		${FileFormatsDir}/PtxFile.cpp
			  ${FileFormatsDir}/ConvertFormat.h		${FileFormatsDir}/ConvertFormat.cpp)
			  
set(DisplaySources	${DisplayDir}/Display.h			 ${DisplayDir}/Display.cpp
			${DisplayDir}/OpenGLinit.h		${DisplayDir}/OpenGLinit.cpp
			${DisplayDir}/zpr.h			${DisplayDir}/zpr.c
			)

INCLUDE_DIRECTORIES(${GeometryDir}
/usr/include/GL/)
			
LINK_DIRECTORIES(/usr/lib)

ADD_EXECUTABLE(Test Test.cpp
Point.h Point.cpp
Vector.h Vector.cpp
MouseClass.h

#${DisplaySources}
# ${FileFormatSources}
			  ${GeometrySources}
			  )

TARGET_LINK_LIBRARIES(Test glut GLU GL Xmu X11)

ADD_EXECUTABLE(Other Other.cpp)
TARGET_LINK_LIBRARIES(Other glut GLU GL Xmu X11)