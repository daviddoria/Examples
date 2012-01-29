cmake_minimum_required(VERSION 2.6)

PROJECT(FindingEigen3WithCMake)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})

FIND_PACKAGE(Eigen3 REQUIRED) #requires you to copy FindEigen3.cmake into the source directory
include_directories(${EIGEN3_INCLUDE_DIR})

FIND_PACKAGE(Umfpack REQUIRED) #requires you to copy FindUmfpack.cmake into the source directory

include_directories(${UMFPACK_INCLUDES} ${EIGEN3_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR}/unsupported/)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET")

ADD_EXECUTABLE(FindingEigen3WithCMake FindingEigen3WithCMake.cpp )
target_link_libraries(FindingEigen3WithCMake umfpack)

# Install the eigen3 package (e.g. set CMAKE_INSTALL_PREFIX=/home/doriad/bin) and then run 'make' and 'make install'

# Then set the CMAKE_INSTALL_PREFIX variable of the application that is using eigen3 to the same directory. (This seems erroneous?)

# Need to set EIGEN3_INCLUDE_DIR in this project CMake settings.

# These packages were installed when this was tested, but they may not be required:
# liblapack-dev
# libblas*
# libatlas*
# libsuitesparse-dev