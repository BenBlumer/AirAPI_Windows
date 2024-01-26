cmake_minimum_required(VERSION 3.22)
project(AIRAPI)
add_subdirectory(deps/hidapi)
add_subdirectory(deps/Fusion)
add_executable(AIRAPI AirAPI.cpp)

target_include_directories(AIRAPI PRIVATE 
        ${CMAKE_SOURCE_DIR}/deps/hidapi/hidapi
        ${CMAKE_SOURCE_DIR}/deps/Fusion/Fusion
        )

target_link_libraries(AIRAPI
hidapi::hidapi 
Fusion 
        )