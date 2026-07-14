# Исполняемые модули (этап 6). Включаются опцией GLONASS_BUILD_APPS в корневом CMakeLists.
# iq_sink.h — header-only в include/glonass/, доступен транзитивно через glonass_fdma_core
# (target_include_directories(... PUBLIC include)) ⇒ отдельной цели-библиотеки не требуется.
add_executable(glonass_signal_gen signal_gen/main.cpp)
target_link_libraries(glonass_signal_gen PRIVATE glonass_fdma_core)
target_compile_features(glonass_signal_gen PRIVATE cxx_std_20)
