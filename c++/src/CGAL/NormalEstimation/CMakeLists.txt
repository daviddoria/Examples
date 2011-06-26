cmake_minimum_required(VERSION 2.6)

Project(NormalEstimation)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")

#########################################3
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

##################
INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} 
/home/doriad/src/CGAL-3.5/include
)

##################
ADD_EXECUTABLE(NormalEstimation NormalEstimation.cpp)
TARGET_LINK_LIBRARIES(NormalEstimation CGAL vtkHybrid)
