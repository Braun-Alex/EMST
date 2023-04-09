cmake_minimum_required(VERSION 3.20)
project(EMST)

set(CMAKE_CXX_STANDARD 23)
set(SOURCES main.cpp)

add_executable(${PROJECT_NAME} ${SOURCES})
