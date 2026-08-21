# Включаются опцией GLONASS_BUILD_APPS в корневом CMakeLists.
add_executable(glonass_signal_gen signal_gen/main.cpp)
target_link_libraries(glonass_signal_gen PRIVATE glonass_fdma_core)
target_compile_features(glonass_signal_gen PRIVATE cxx_std_20)

add_library(glonass_params_l1oc INTERFACE)
target_include_directories(glonass_params_l1oc INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
    $<TARGET_PROPERTY:glonass_fdma_core,INTERFACE_INCLUDE_DIRECTORIES>)

# Тракт L1OC отдельный модуль запуска: тракты не смешиваются, состав ключей иной
# (--j вместо --k, --band неприменим)
add_executable(glonass_signal_gen_l1oc signal_gen_l1oc/main.cpp)
target_link_libraries(glonass_signal_gen_l1oc PRIVATE glonass_fdma_core glonass_params_l1oc)
target_compile_features(glonass_signal_gen_l1oc PRIVATE cxx_std_20)

# Сопряжение с внешним программным приёмником: CF32 LE -> CS16 LE
add_executable(glonass_iq_convert iq_convert/main.cpp)
target_compile_features(glonass_iq_convert PRIVATE cxx_std_20)

# Микросервис L1OC включается опцией GLONASS_BUILD_SERVICE в корневом CMakeLists.
if(GLONASS_BUILD_SERVICE)
    find_package(Threads REQUIRED)

    add_library(cpp_httplib INTERFACE)
    target_include_directories(cpp_httplib SYSTEM INTERFACE
        ${CMAKE_SOURCE_DIR}/third_party/cpp-httplib)
    target_link_libraries(cpp_httplib INTERFACE Threads::Threads)

    add_library(glonass_service_l1oc STATIC
        signal_service_l1oc/service.h
        signal_service_l1oc/service.cpp
        signal_service_l1oc/service_config.h
        signal_service_l1oc/service_version.h
        signal_service_l1oc/error_response.h
        signal_service_l1oc/json_writer.h
        signal_service_l1oc/state_metrics.h
        signal_service_l1oc/state_metrics.cpp
        signal_service_l1oc/stream_session.h
        signal_service_l1oc/stream_session.cpp
        signal_service_l1oc/stream_slot.h
        signal_service_l1oc/fft_radix2.h
        signal_service_l1oc/svg_canvas.h
        signal_service_l1oc/svg_canvas.cpp
        signal_service_l1oc/frame_psd.h
        signal_service_l1oc/frame_psd.cpp
        signal_service_l1oc/tcp_stream_session.h
        signal_service_l1oc/tcp_stream_session.cpp)
    # iq_convert — примитивы квантования CS16, общие с конвертером записи (решение 6)
    target_include_directories(glonass_service_l1oc PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/signal_service_l1oc
        ${CMAKE_CURRENT_SOURCE_DIR}/iq_convert)

    target_link_libraries(glonass_service_l1oc PUBLIC cpp_httplib glonass_fdma_core
                                                      glonass_params_l1oc)
    target_compile_features(glonass_service_l1oc PUBLIC cxx_std_20)

    add_executable(glonass_signal_service_l1oc signal_service_l1oc/main.cpp)
    target_link_libraries(glonass_signal_service_l1oc PRIVATE glonass_service_l1oc)
    target_compile_features(glonass_signal_service_l1oc PRIVATE cxx_std_20)
endif()
