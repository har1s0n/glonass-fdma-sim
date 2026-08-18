#ifndef SERVICE_VERSION_H
#define SERVICE_VERSION_H

// сведения, выдаваемые точкой GET /v1/info
namespace glonass_service {
inline constexpr const char* serviceName    = "signal-service-l1oc";
inline constexpr const char* serviceVersion = "1.0";
inline constexpr const char* apiVersion     = "v1";
inline constexpr const char* band           = "L1OC";
inline constexpr const char* icdProfile     = "ГЛОНАСС L1OC ред. 1.0 (2016)";
} // namespace glonass_service

#endif // SERVICE_VERSION_H
