Project(VectorOfArray)

FIND_PACKAGE(VXL REQUIRED)

INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

ADD_EXECUTABLE(VectorOfArray VectorOfArray.cpp)


