cmake_minimum_required(VERSION 2.6)

Project(VILBasics)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

ADD_EXECUTABLE(VILBasics VILBasics.cpp)
TARGET_LINK_LIBRARIES(VILBasics
vil
rrel
)
