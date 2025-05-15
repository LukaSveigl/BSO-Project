cmake_minimum_required(VERSION 3.31)
project(BSO_Project C)

set(CMAKE_C_STANDARD 11)

include_directories(include)

add_executable(BSO_Project
        include/comms.h
        include/election.h
        include/sensing.h
        include/defs.h
        main.cpp)
