#ifndef WSPR_TRANSMIT_TYPES_HPP
#define WSPR_TRANSMIT_TYPES_HPP

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

#endif
