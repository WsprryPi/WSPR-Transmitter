/**
 * @file wspr_transmit_types.hpp
 * @brief Shared transmission state and backend-neutral configuration types.
 *
 * Copyright © 2025 - 2026 Lee C. Bussy (@LBussy). All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef WSPR_TRANSMIT_TYPES_HPP
#define WSPR_TRANSMIT_TYPES_HPP

#include <cstddef>
#include <cstdint>
#include <string>

/**
 * @enum WsprTransmitState
 * @brief High-level runtime state for the transmitter controller.
 *
 * @details
 * The state machine is owned by the controller and observed by the active
 * backend. Values describe operational readiness and fault handling rather
 * than low-level hardware state.
 */
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

/**
 * @struct WsprTransmissionPlan
 * @brief Backend-neutral snapshot of transmission intent and configuration.
 *
 * @details
 * The controller constructs this lightweight plan from its active
 * configuration and passes it to a backend when preparing, configuring, or
 * emitting a transmission. It contains only the data a backend should need to
 * render RF, without exposing backend-private runtime state such as DMA
 * cursors, watchdog state, mailbox handles, or recovery bookkeeping.
 */
struct WsprTransmissionPlan
{
    /**
     * @brief Requested RF center frequency in hertz (Hz).
     */
    double frequency_hz = 0.0;

    /**
     * @brief Tone spacing in hertz (Hz) between adjacent WSPR tones.
     */
    double tone_spacing_hz = 0.0;

    /**
     * @brief Logical output power level index used by the active backend.
     *
     * @details
     * This is not a calibrated power value in milliwatts (mW) or dBm. Each
     * backend interprets the level according to its hardware-specific output
     * control model.
     */
    int power_level = 0;

    /**
     * @brief Total number of symbols that will be emitted for this transmission.
     *
     * @details
     * Tone mode sets this to zero because it is open-ended. Prepared WSPR mode
     * reports the total symbol count across all encoded frames.
     */
    std::size_t total_symbol_count = 0;

    std::size_t symbolCount() const noexcept
    {
        return total_symbol_count;
    }

};

/**
 * @struct WsprTransmissionConfigureResult
 * @brief Result returned by a backend after applying transmission setup.
 *
 * @details
 * This structure reports the small amount of backend-neutral information the
 * controller may need after hardware-specific configuration has completed.
 * It intentionally avoids exposing backend internals while still allowing the
 * controller to reflect adjusted transmit parameters back to callers.
 */
struct WsprTransmissionConfigureResult
{
    /**
     * @brief Actual RF center frequency applied by the backend in hertz (Hz).
     *
     * @details
     * This may differ from the requested plan frequency when hardware tuning
     * resolution requires quantization or a nearby realizable value.
     */
    double applied_frequency_hz = 0.0;
};

#endif
