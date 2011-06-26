cmake_minimum_required(VERSION 2.6)

PROJECT(InstallTest)

ADD_EXECUTABLE(InstallTest InstallTest.cxx)

# Install to a hard coded path
#install(TARGETS InstallTest RUNTIME DESTINATION /home/doriad/Desktop/InstallTest)

# Install to a path specified in ccmake
install(TARGETS InstallTest RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})
