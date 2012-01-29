cmake_minimum_required(VERSION 2.6)

PROJECT(QWTDemo)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})

#QT4_WRAP_UI(UISrcs form.ui)
#QT4_WRAP_CPP(MOCSrcs form.h)

include_directories(${include_directories} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} /home/doriad/src/qwt-6.0/src)
link_directories(/home/doriad/src/qwt-6.0/lib)

#ADD_EXECUTABLE(QWT main.cpp ${MOCSrcs} ${UISrcs})
ADD_EXECUTABLE(QWTDemo main.cpp)
TARGET_LINK_LIBRARIES(QWTDemo ${QT_LIBRARIES} qwt)

