Project(SparseLeastSquares)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

include_directories( ${VXL_VCL_INCLUDE_DIR} )
include_directories( ${VXL_CORE_INCLUDE_DIR} )

ADD_EXECUTABLE(SparseLeastSquares SparseLeastSquares.cpp)
TARGET_LINK_LIBRARIES(SparseLeastSquares vnl rrel)
