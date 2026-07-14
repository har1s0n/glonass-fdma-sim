cmake_minimum_required(VERSION 3.20)
project(glonass_fdma_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Статическая библиотека ядра
add_library(glonass_fdma_core STATIC
    include/glonass/nav_message.h
    include/glonass/carrier_nco.h
    include/glonass/modulation.h
    include/glonass/signal_combine.h
    include/glonass/signal_source.h
    include/glonass/source_config.h
    include/glonass/numeric.h
    include/glonass/types.h
    include/glonass/code_phase.h
    include/glonass/iq_sink.h
    include/glonass/ranging_code.h
    src/ranging_code.cpp
    src/code_phase.cpp
    src/nav_message.cpp
    src/carrier_nco.cpp
    src/signal_source.cpp)

target_include_directories(glonass_fdma_core PUBLIC include)
target_compile_features(glonass_fdma_core PUBLIC cxx_std_20)

option(GLONASS_BUILD_APPS "Собирать исполняемые модули (apps/)" ON)
if(GLONASS_BUILD_APPS)
   add_subdirectory(apps)
endif()

option(GLONASS_BUILD_TESTS "Build unit tests" ON)
if(GLONASS_BUILD_TESTS)
    include(CTest)
    enable_testing()
    add_subdirectory(tests)
endif()
