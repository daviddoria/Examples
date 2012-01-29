cmake_minimum_required(VERSION 2.6)

PROJECT(DecomposeProjectionMatrix)

FIND_PACKAGE( OpenCV REQUIRED )
INCLUDE_DIRECTORIES( ${OPENCV_INCLUDE_DIR} )

ADD_EXECUTABLE(DecomposeProjectionMatrix DecomposeProjectionMatrix.cxx)
TARGET_LINK_LIBRARIES(DecomposeProjectionMatrix ${OPENCV_LIBRARIES}
opencv_core opencv_highgui
opencv_flann opencv_imgproc opencv_highgui opencv_ml opencv_video opencv_objdetect
          opencv_features2d opencv_calib3d opencv_legacy opencv_contrib
)

