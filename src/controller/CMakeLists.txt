project(controller)
cmake_minimum_required(VERSION 3.22)

include_directories(include
                    ../simulator/include)
aux_source_directory(src SRC)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library(controller SHARED ${SRC})
target_link_libraries(controller ${EIGEN3_LIBRARY})