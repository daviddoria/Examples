cmake_minimum_required(VERSION 2.6)

PROJECT(RandomImageSource)

find_package(ItkVtkGlue REQUIRED)
include(${ItkVtkGlue_USE_FILE})

add_executable(RandomImageSource RandomImageSource.cxx)
target_link_libraries(RandomImageSource
  ItkVtkGlue  ${VTK_LIBRARIES} ${ITK_LIBRARIES})