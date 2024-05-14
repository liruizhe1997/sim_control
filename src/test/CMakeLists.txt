project(test_ode1)
cmake_minimum_required(VERSION 3.22)

set(CMAKE_BUILD_TYPE "Debug")

include_directories(include
                    ../simulator/include
                    ../controller/include)
aux_source_directory(src SRC)

add_executable(test_simulator src/test_simulator.cpp)
target_link_libraries(test_simulator simulator)
target_link_libraries(test_simulator controller)