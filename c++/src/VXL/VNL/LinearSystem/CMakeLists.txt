Project(LinearSystem)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

ADD_EXECUTABLE(LinearSystem LinearSystem.cpp)


TARGET_LINK_LIBRARIES(LinearSystem vnl rrel)
