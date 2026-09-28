cmake_minimum_required(VERSION 3.16)

project(LECTURE_2)

# 告诉 CMake 需要 OpenCV，找不到会直接报错
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(part_1/includes part_2/ex3)

add_executable(cmake_learn
    part_1/src/cmake_learn.cpp
    part_1/tools/hello.cpp
)

add_executable(ex1 part_2/ex1/example1.cpp)
target_link_libraries(ex1 ${OpenCV_LIBS})

add_executable(ex2 part_2/ex2/example2.cpp)
target_link_libraries(ex2 ${OpenCV_LIBS})

add_executable(ex3 part_2/ex3/car.cpp part_2/ex3/ex3.cpp)
target_link_libraries(ex3 ${OpenCV_LIBS})
