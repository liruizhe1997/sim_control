project(simulator)
cmake_minimum_required(VERSION 3.22)

include_directories(include)
aux_source_directory(src SRC)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library(simulator SHARED ${SRC})
target_link_libraries(simulator ${EIGEN3_LIBRARY})