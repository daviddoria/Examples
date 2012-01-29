cmake_minimum_required(VERSION 2.6)

Project(VNLMinimize)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

ADD_EXECUTABLE(LMTest LMTest.cpp)
TARGET_LINK_LIBRARIES(LMTest 
vnl 
rrel
)

ADD_EXECUTABLE(CostTest CostTest.cpp)
TARGET_LINK_LIBRARIES(CostTest vnl 
rrel
)
