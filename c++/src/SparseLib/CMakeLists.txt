cmake_minimum_required(VERSION 2.6)

Project(SparseLib)

INCLUDE_DIRECTORIES(/home/doriad/src/SparseLib++/1.7/include)
INCLUDE_DIRECTORIES(/home/doriad/src/SparseLib++/1.7/mv/include)

LINK_DIRECTORIES(${LINK_DIRECTORIES} /home/doriad/src/SparseLib++/1.7/lib)

ADD_EXECUTABLE(SparseLib SparseLib.cpp)
TARGET_LINK_LIBRARIES(SparseLib mv
sparse  
spblas
)
