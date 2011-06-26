Project(LinearLeastSquares)

FIND_PACKAGE(VXL REQUIRED)

INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

ADD_EXECUTABLE(LinearLeastSquares LinearLeastSquares.cpp)


TARGET_LINK_LIBRARIES(LinearLeastSquares vnl rrel)
