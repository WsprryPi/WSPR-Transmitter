#pragma once

#include <array>
#include <cmath>
#include <string>
#include <string_view>

#include "execution_plan.hpp"
#include "transmission_request.hpp"

namespace wsprrypi
{

struct GpioBandPolicyDecision
{
    bool allowed{true};
    std::string band;
    std::string error;
};

inline GpioBandPolicyDecision evaluate_gpio_band_policy(
    BackendKind backend,
    double frequency_hz)
{
    if (backend != BackendKind::RPI_CLOCK_GPIO ||
        !std::isfinite(frequency_hz) || frequency_hz <= 0.0)
    {
        return {};
    }

    struct DisqualifiedBand
    {
        std::string_view name;
        double lower_hz;
        double upper_hz;
    };

    // These ranges intentionally match WsprryPi's canonical HamBand lookup.
    static constexpr std::array<DisqualifiedBand, 3> disqualified_bands{{
        {"12 m", 24890000.0, 24990000.0},
        {"6 m", 50000000.0, 52000000.0},
        {"2 m", 144000000.0, 148000000.0},
    }};

    for (const auto& band : disqualified_bands)
    {
        if (frequency_hz < band.lower_hz || frequency_hz > band.upper_hz)
            continue;

        GpioBandPolicyDecision decision;
        decision.allowed = false;
        decision.band = std::string(band.name);
        decision.error =
            "Direct GPIO transmission is blocked on the " + decision.band +
            " band because that band is not qualified for usable output. "
            "This restriction applies only to direct GPIO; select the Si5351 "
            "backend or choose a GPIO-qualified band.";
        return decision;
    }

    return {};
}

inline GpioBandPolicyDecision evaluate_gpio_band_policy(
    const ExecutionPlan& plan)
{
    if (plan.backend != BackendKind::RPI_CLOCK_GPIO)
        return {};

    for (const auto& event : plan.events)
    {
        if (!event.rf_on && event.type != RfEventType::SET_FREQUENCY)
            continue;

        const auto decision = evaluate_gpio_band_policy(
            plan.backend,
            event.frequency_hz);
        if (!decision.allowed)
            return decision;
    }

    return {};
}

} // namespace wsprrypi
