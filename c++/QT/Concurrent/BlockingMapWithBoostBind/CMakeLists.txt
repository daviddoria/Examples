cmake_minimum_required(VERSION 2.6)

PROJECT(BlockingMapWithBoostBind)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

ADD_EXECUTABLE(BlockingMapWithBoostBind BlockingMapWithBoostBind.cpp)# ${MOCSrcs} ${UISrcs})
TARGET_LINK_LIBRARIES(BlockingMapWithBoostBind ${QT_LIBRARIES})
