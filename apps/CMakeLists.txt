# Включаются опцией GLONASS_BUILD_APPS в корневом CMakeLists.
add_executable(glonass_signal_gen signal_gen/main.cpp)
target_link_libraries(glonass_signal_gen PRIVATE glonass_fdma_core)
target_compile_features(glonass_signal_gen PRIVATE cxx_std_20)

# Тракт L1OC — отдельный модуль запуска: тракты не смешиваются, состав ключей иной
# (--j вместо --k, --band неприменим)
add_executable(glonass_signal_gen_l1oc signal_gen_l1oc/main.cpp)
target_link_libraries(glonass_signal_gen_l1oc PRIVATE glonass_fdma_core)
target_compile_features(glonass_signal_gen_l1oc PRIVATE cxx_std_20)

# Сопряжение с внешним программным приёмником: CF32 LE -> CS16 LE
add_executable(glonass_iq_convert iq_convert/main.cpp)
target_compile_features(glonass_iq_convert PRIVATE cxx_std_20)
