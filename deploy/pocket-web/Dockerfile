# Образ приёмника стенда: приложение pocket_web из PocketSDR 03787da5 (third_party/PocketSDR)
# Контекст сборки: корень репозитория (deploy/compose.yaml).
# libsdr на x86_64 собирается флагами исходника (AVX2): процессор стенда должен поддерживать AVX2.
# На aarch64 флаги исходника -DNEON -mcpu=native не годятся: GCC 12 не опознаёт Apple, и
# без расширения dotprod не компилируется vdotq_s32 (sdr_func.c). Там задаётся явно
# -DNEON -march=armv8.2-a+dotprod: процессор должен поддерживать ARMv8.2 с dotprod.

FROM debian:bookworm-slim AS build
RUN apt-get update && apt-get install -y --no-install-recommends \
        g++ make libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src/PocketSDR
COPY third_party/PocketSDR/ ./
# Библиотеки librtk, libpocketfft и libsdr устанавливаются в lib/linux; SoapySDR и FFTW не применяются.
# Порядок тот же, что в lib/build/makefile; libsdr.mk вызывается отдельно, чтобы переопределение
# OPTIONS на aarch64 не затронуло librtk и libpocketfft
RUN cd lib/build \
    && make -f librtk.mk \
    && make -f libpocketfft.mk \
    && if [ "$(uname -m)" = aarch64 ]; then \
           make -f libsdr.mk USE_SOAPY=0 USE_FFTW=0 OPTIONS="-DNEON -march=armv8.2-a+dotprod"; \
       else \
           make -f libsdr.mk USE_SOAPY=0 USE_FFTW=0; \
       fi \
    && make -f librtk.mk install \
    && make -f libpocketfft.mk install \
    && make -f libsdr.mk install \
    && make -C ../../app/pocket_web USE_SOAPY=0 USE_FFTW=0

FROM debian:bookworm-slim AS runtime
RUN apt-get update && apt-get install -y --no-install-recommends libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/* \
    && useradd --system --uid 10002 pocketweb \
    && mkdir -p /usr/share/pocket-web/html /var/lib/pocket-web
COPY --from=build /src/PocketSDR/app/pocket_web/pocket_web /usr/local/bin/
COPY deploy/pocket-web/pocket_web.ini /etc/pocket-web/pocket_web.ini
COPY deploy/pocket-web/healthcheck.sh /usr/local/bin/pocket-web-healthcheck
# Рабочий каталог принадлежит root и закрыт на запись: навигационные данные приёмника
# (.pocket_navdata.csv) между сеансами не переносятся, файл настроек не перезаписывается
WORKDIR /var/lib/pocket-web
USER pocketweb
EXPOSE 8081
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
    CMD ["/usr/local/bin/pocket-web-healthcheck"]
# Каталог -html пуст: интерфейс PocketSDR не отдаётся, корень отвечает 404, WebSocket /ws работает
ENTRYPOINT ["/usr/local/bin/pocket_web", "-web", "0.0.0.0:8081", \
            "-html", "/usr/share/pocket-web/html", \
            "-ini", "/etc/pocket-web/pocket_web.ini", "-start"]
