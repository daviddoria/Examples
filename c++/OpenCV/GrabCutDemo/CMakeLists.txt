cmake_minimum_required(VERSION 2.6)

PROJECT(GrabCutDemo)

FIND_PACKAGE( OpenCV REQUIRED )
INCLUDE_DIRECTORIES( ${OPENCV_INCLUDE_DIR} )

ADD_EXECUTABLE(GrabCutDemo GrabCutDemo.cpp)
TARGET_LINK_LIBRARIES(GrabCutDemo ${OPENCV_LIBRARIES} cv cxcore highgui ml cvaux)

