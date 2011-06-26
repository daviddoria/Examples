cmake_minimum_required(VERSION 2.6)

PROJECT(CheckerBoardCalibration)

FIND_PACKAGE(OpenCV REQUIRED )
INCLUDE_DIRECTORIES( ${OPENCV_INCLUDE_DIR} )

ADD_EXECUTABLE(CheckerBoardCalibration CheckerBoardCalibration.cxx)
TARGET_LINK_LIBRARIES(CheckerBoardCalibration ${OpenCV_LIBS})

ADD_EXECUTABLE(HasCheckerboard HasCheckerboard.cxx)
TARGET_LINK_LIBRARIES(HasCheckerboard ${OpenCV_LIBS})