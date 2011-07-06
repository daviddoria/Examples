cmake_minimum_required(VERSION 2.6)

PROJECT(ImagePCAShapeModelEstimator)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

ADD_EXECUTABLE(ImagePCAShapeModelEstimator ImagePCAShapeModelEstimator.cxx)
TARGET_LINK_LIBRARIES(ImagePCAShapeModelEstimator ${ITK_LIBRARIES})
