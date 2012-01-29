cmake_minimum_required(VERSION 2.6)

PROJECT(LevenbergMarquardt)
include_directories(/media/portable/src/Eigen)
# FIND_PACKAGE(Eigen3 REQUIRED)
# INCLUDE(${Eigen_USE_FILE})

include_directories(${EIGEN3_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR}/unsupported/)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET")

ADD_EXECUTABLE(AnalyticDerivative AnalyticDerivative.cpp )

ADD_EXECUTABLE(NumericalDerivative NumericalDerivative.cpp )

ADD_EXECUTABLE(CurveFitting CurveFitting.cpp )
