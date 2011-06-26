cmake_minimum_required(VERSION 2.6)

Project(SurfaceReconstruction)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")

#########################################3
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

##################
INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} 
/home/doriad/src/CGAL-3.5/include
)

##################
ADD_EXECUTABLE(SurfaceReconstruction SurfaceReconstruction.cpp)
TARGET_LINK_LIBRARIES(SurfaceReconstruction 
CGAL taucs lapack metis gmp mpfr pthread boost_thread-mt vtkHybrid)

##################
ADD_EXECUTABLE(SurfaceFromPointCloud SurfaceFromPointCloud.cpp)
TARGET_LINK_LIBRARIES(SurfaceFromPointCloud
CGAL taucs lapack metis gmp mpfr boost_thread-mt vtkHybrid)
