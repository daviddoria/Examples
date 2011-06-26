cmake_minimum_required(VERSION 2.6)

PROJECT(GrabFrames)

FIND_PACKAGE(OpenCV REQUIRED )
INCLUDE_DIRECTORIES( ${OPENCV_INCLUDE_DIR} )

ADD_EXECUTABLE(GrabFrames GrabFrames.cxx)
#TARGET_LINK_LIBRARIES(GrabFrames ${OPENCV_LIBRARIES})
TARGET_LINK_LIBRARIES(GrabFrames opencv_core opencv_highgui)

ADD_EXECUTABLE(SaveSingleFrame SaveSingleFrame.cxx)
TARGET_LINK_LIBRARIES(SaveSingleFrame opencv_core opencv_highgui)
