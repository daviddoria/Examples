cmake_minimum_required(VERSION 2.6)

PROJECT(Image)

find_package(wxWidgets COMPONENTS core base REQUIRED)
include( ${wxWidgets_USE_FILE} )

ADD_EXECUTABLE(Image Image.cpp )
TARGET_LINK_LIBRARIES(Image ${wxWidgets_LIBRARIES})
