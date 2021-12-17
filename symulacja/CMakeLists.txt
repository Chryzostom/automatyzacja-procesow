cmake_minimum_required(VERSION 3.10)
set (CMAKE_GENERATOR "Unix Makefiles" CACHE INTERNAL "" FORCE)
set(CMAKE_FORCE_C_COMPILER "C:/MinGW/bin/x86_64-w64-mingw32-gcc.exe")
set(CMAKE_FORCE_CXX_COMPILER "C:/MinGW/bin/x86_64-w64-mingw32-g++.exe")
set(CMAKE_FIND_ROOT_PATH "C:/Users/mateo/OneDrive/Desktop/projekt/symulacja")

# add_compile_options(-Wall -Wextra -Wpedantic)

file(GLOB SOURCES
    ./*.hpp
    ./*.cpp
)

# set the project name
project(symulacja)

unset(MUJOCO_INCLUDE_DIR CACHE)
unset(GLFW3_INCLUDE_DIR CACHE)

# searching for include director
find_path(MUJOCO_INCLUDE_DIR
            NAMES "mujoco.h"
            PATHS "mujoco210/include")
find_path(GLFW3_INCLUDE_DIR 
            NAMES "glfw3.h"
            PATHS "mujoco210/include")

# searching for library file
find_library(MUJOCO_LIBRARY 
            NAMES "mujoco210.lib"
            PATHS "mujoco210/bin")  
find_library(GLFW3_LIBRARY            
            NAMES "glfw3.lib"
            PATHS "mujoco210/bin")  

if (MUJOCO_INCLUDE_DIR AND MUJOCO_LIBRARY)
    # you may need that if further action in your CMakeLists.txt depends
    # on detecting your library
    set(SIFTGPU_FOUND TRUE)
    # you may need that if you want to conditionally compile some parts
    # of your code depending on library availability
    add_definitions(-DHAVE_LIBMUJOCO=1)
    # those two, you really need
    include_directories(${MUJOCO_INCLUDE_DIR})
    set(LIB_MUJOCO ${LIB_MUJOCO} ${MUJOCO_LIBRARY})
endif ()

message(STATUS "MuJoCo include: \"${MUJOCO_INCLUDE_DIR}\"")
message(STATUS "MuJoCo library: \"${LIB_MUJOCO}\"")

if (GLFW3_INCLUDE_DIR AND GLFW3_LIBRARY)
    # you may need that if further action in your CMakeLists.txt depends
    # on detecting your library
    set(SIFTGPU_FOUND TRUE)
    # you may need that if you want to conditionally compile some parts
    # of your code depending on library availability
    add_definitions(-DHAVE_LIBGLFW3=1)
    # those two, you really need
    include_directories(${GLFW3_INCLUDE_DIR})
    set(LIB_GLFW3 ${LIB_GLFW3} ${GLFW3_LIBRARY})
endif ()

message(STATUS "glfw3 include: \"${GLFW3_INCLUDE_DIR}\"")
message(STATUS "glfw3 library: \"${LIB_GLFW3}\"")

set(CMAKE_BUILD_TYPE "Debug")

# add the executable
add_executable(main 
    ${SOURCES}
)
target_link_libraries(main
    ${LIB_GLFW3}
    ${LIB_MUJOCO}
)
