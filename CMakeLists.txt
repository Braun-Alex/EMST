cmake_minimum_required(VERSION 3.20)
project(Computation)

set(CMAKE_CXX_STANDARD 23)
set(HEADERS point.h edge.h functions.h)
set(SOURCES main.cpp point.cpp edge.cpp functions.cpp)

add_executable(${PROJECT_NAME} ${SOURCES})
