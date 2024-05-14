project(test)
cmake_minimum_required(VERSION 3.22)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../lib)

add_subdirectory(simulator)
add_subdirectory(controller)
add_subdirectory(test)