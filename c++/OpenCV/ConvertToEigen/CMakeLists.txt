cmake_minimum_required(VERSION 2.6)

PROJECT(ConvertToEigen)

# OpenCV
FIND_PACKAGE(OpenCV REQUIRED )
INCLUDE_DIRECTORIES( ${OPENCV_INCLUDE_DIR} )

# Eigen
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR} ${CMAKE_MODULE_PATH})

FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

# Executables
ADD_EXECUTABLE(ConvertToEigen ConvertToEigen.cxx)
TARGET_LINK_LIBRARIES(ConvertToEigen opencv_core opencv_highgui
opencv_flann opencv_imgproc
#opencv_highgui
#opencv_ml
#opencv_video
#opencv_objdetect
#opencv_features2d
#opencv_calib3d
#opencv_legacy
#opencv_contrib
)
