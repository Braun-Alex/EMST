cmake_minimum_required(VERSION 3.20)
project(Computation)

set(CMAKE_CXX_STANDARD 23)
set(HEADERS point.h edge.h functions.h tests.h)
set(SOURCES main.cpp point.cpp edge.cpp functions.cpp tests.cpp)

add_subdirectory(testing/googletest)

add_executable(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_link_libraries(${PROJECT_NAME} gtest_main gtest gmock)