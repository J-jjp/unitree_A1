cmake_minimum_required(VERSION 3.10)
 
project(serial_port)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
file(GLOB DOG_SRCS 
"src/control/*.cpp"
"main.cpp")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/communication 
        ${CMAKE_CURRENT_SOURCE_DIR}/include/control)
 
 
add_executable(${PROJECT_NAME} ${DOG_SRCS})
 
target_link_libraries(${PROJECT_NAME} m )