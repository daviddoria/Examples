cmake_minimum_required(VERSION 2.6)

PROJECT(OctaveTest)

INCLUDE_DIRECTORIES(/usr/include/ 
/usr/local/include/ 
/usr/include/octave-3.2.4/
)

LINK_DIRECTORIES(/usr/lib /usr/local/lib
#/usr/lib/octave-3.0.2/
#/usr/lib/octave-3.0.1/
/usr/lib/octave-3.2.4/
)

ADD_EXECUTABLE(OctaveTest OctaveTest.cpp)

TARGET_LINK_LIBRARIES(OctaveTest  
octave octinterp
)
