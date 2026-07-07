cmake_minimum_required(VERSION 3.20)
project(glonass_fdma_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Статическая библиотека ядра
add_library(glonass_fdma_core STATIC
    src/ranging_code.cpp
    src/code_phase.cpp
    src/nav_message.cpp
    include/glonass/nav_message.h
    include/glonass/carrier_nco.h
    src/carrier_nco.cpp)

target_include_directories(glonass_fdma_core PUBLIC include)
target_compile_features(glonass_fdma_core PUBLIC cxx_std_20)

option(GLONASS_BUILD_TESTS "Build unit tests" ON)
if(GLONASS_BUILD_TESTS)
    include(CTest)
    enable_testing()
    add_subdirectory(tests)
endif()
