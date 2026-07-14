include(FetchContent)

set(GTEST_URL "https://github.com/google/googletest/archive/f8d7d77c06936315286eb55f8de22cd23c188571.zip")

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
    FetchContent_Declare(googletest URL "${GTEST_URL}" DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
else()
    FetchContent_Declare(googletest URL "${GTEST_URL}")
endif()

FetchContent_MakeAvailable(googletest)

function(glonass_add_gtest TARGET_NAME)
    cmake_parse_arguments(T "" "" "SRCS;LIBS" ${ARGN})
    add_executable(${TARGET_NAME} ${T_SRCS})
    target_link_libraries(${TARGET_NAME} PRIVATE
        GTest::gtest GTest::gtest_main glonass_fdma_core ${T_LIBS})
    target_compile_features(${TARGET_NAME} PRIVATE cxx_std_20)
    include(GoogleTest)
    gtest_discover_tests(${TARGET_NAME})
endfunction()

glonass_add_gtest(test_ranging_code SRCS test_ranging_code.cpp)
glonass_add_gtest(test_nav_message SRCS test_nav_message.cpp)
glonass_add_gtest(test_carrier_nco SRCS test_carrier_nco.cpp)
glonass_add_gtest(test_modulation SRCS test_modulation.cpp)
glonass_add_gtest(test_signal_source SRCS test_signal_source.cpp)
glonass_add_gtest(test_iq_export SRCS test_iq_export.cpp)


target_compile_definitions(test_ranging_code PRIVATE
    GLONASS_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/data")
target_compile_definitions(test_nav_message PRIVATE
    GLONASS_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/data")
