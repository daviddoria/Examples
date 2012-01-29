cmake_minimum_required(VERSION 2.6)

#INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} $ENV{CPLUS_INCLUDE_PATH})

INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} /usr/lib/qt-3.3/include/)
#INCLUDE_DIRECTORIES($ENV{CPLUS_INCLUDE_PATH})

LINK_DIRECTORIES(${LINK_DIRECTORIES} /usr/lib/qt-3.3/lib/)

PROJECT(Basics)

########################
ADD_EXECUTABLE(Basics Basics.cpp )
TARGET_LINK_LIBRARIES(Basics qt-mt)
#TARGET_LINK_LIBRARIES(Basics qui)

########################
ADD_EXECUTABLE(ButtonCallback ButtonCallback.cpp )
TARGET_LINK_LIBRARIES(ButtonCallback qt-mt)

########################
ADD_EXECUTABLE(Slider Slider.cpp )
TARGET_LINK_LIBRARIES(Slider qt-mt)
