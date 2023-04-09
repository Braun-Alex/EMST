cmake_minimum_required(VERSION 3.20)
project(EMST)

set(CMAKE_CXX_STANDARD 23)
set(HEADERS point.h)
set(SOURCES main.cpp point.cpp)

add_executable(${PROJECT_NAME} ${SOURCES})
