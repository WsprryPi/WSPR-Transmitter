#ifndef WSPR_TRANSMIT_TYPES_HPP
#define WSPR_TRANSMIT_TYPES_HPP

#include <cstddef>

enum class WsprTransmitState
{
    DISABLED,
    ENABLED,
    TRANSMITTING,
    RECOVERING,
    COMPLETE,
    CANCELLED,
    HUNG
};

constexpr const char *wsprTransmitStateToString(WsprTransmitState state) noexcept
{
    switch (state)
    {
    case WsprTransmitState::DISABLED:
        return "DISABLED";
    case WsprTransmitState::ENABLED:
        return "ENABLED";
    case WsprTransmitState::TRANSMITTING:
        return "TRANSMITTING";
    case WsprTransmitState::RECOVERING:
        return "RECOVERING";
    case WsprTransmitState::COMPLETE:
        return "COMPLETE";
    case WsprTransmitState::CANCELLED:
        return "CANCELLED";
    case WsprTransmitState::HUNG:
        return "HUNG";
    default:
        return "UNKNOWN";
    }
}

enum class WsprTransmitLogLevel
{
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
};

enum class WsprTransmissionCallbackEvent
{
    STARTING,
    COMPLETE,
    SKIPPED,
    LOGGING
};

struct WsprTransmissionPlan
{
    double frequency_hz = 0.0;
    double tone_spacing_hz = 0.0;
    int power_level = 0;
    std::size_t symbol_count = 0;
};

#endif
