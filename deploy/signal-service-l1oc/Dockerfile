# Образ микросервиса цифровой модели навигационного сигнала L1OC (БИНК)
# Имя исполняемого файла, переменные окружения и их умолчания взяты из кода
# (apps/CMakeLists.txt, apps/signal_service_l1oc/service_config.h).
# Контекст сборки: корень репозитория (deploy/compose.yaml). Обращений к сети, кроме базового образа
# и пакетов Debian, нет: cpp-httplib в third_party/, тесты не собираются.

FROM debian:bookworm-slim AS build
RUN apt-get update && apt-get install -y --no-install-recommends \
        g++ cmake ninja-build \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY CMakeLists.txt ./
COPY include/ include/
COPY src/ src/
COPY apps/ apps/
COPY third_party/cpp-httplib/ third_party/cpp-httplib/
RUN cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DGLONASS_BUILD_TESTS=OFF \
    && cmake --build build --target glonass_signal_service_l1oc

FROM debian:bookworm-slim AS runtime
RUN useradd --system --uid 10001 glonass
COPY --from=build /src/build/apps/glonass_signal_service_l1oc /usr/local/bin/
USER glonass
ENV SIGNAL_PORT=8080 \
    SIGNAL_MAX_JOBS=1 \
    SIGNAL_MAX_STREAMS=1 \
    SIGNAL_WORK_DIR=/tmp/signal \
    SIGNAL_TCP_PORTS=30070-30079
EXPOSE 8080
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
    CMD ["/usr/local/bin/glonass_signal_service_l1oc", "--healthcheck"]
ENTRYPOINT ["/usr/local/bin/glonass_signal_service_l1oc"]
