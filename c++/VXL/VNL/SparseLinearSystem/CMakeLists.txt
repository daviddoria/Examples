Project(SparseLinearSystem)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

#INCLUDE_DIRECTORIES(/home/doriad/src/vxl/v3p/netlib/)
include_directories( ${VXL_VCL_INCLUDE_DIR} )
include_directories( ${VXL_CORE_INCLUDE_DIR} )

ADD_EXECUTABLE(SparseLinearSystem SparseLinearSystem.cpp)
TARGET_LINK_LIBRARIES(SparseLinearSystem vnl rrel)
