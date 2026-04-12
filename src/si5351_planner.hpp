#ifndef SI5351_PLANNER_HPP
#define SI5351_PLANNER_HPP

#include "si5351_device.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

/**
 * @brief Builds Si5351 register plans for a transmission
 *
 * Converts requested RF frequencies into a backend-specific programming plan
 * suitable for efficient runtime execution.
 *
 * The planner should:
 *
 * - Choose the static PLL parking strategy
 * - Build initial register writes
 * - Precompute per-tone register write sets
 * - Minimize symbol-boundary work
 *
 * The planner should not perform I2C I/O or manage execution timing.
 */
class Si5351Planner
{
public:
    /**
     * @brief Transmission mode understood by the planner
     */
    enum class Mode
    {
        TONE,
        WSPR,
        QRSS,
        FSKCW,
        DFCW
    };

    /**
     * @brief Planner configuration
     */
    struct Config
    {
        std::uint32_t reference_hz = 27000000;
        std::uint64_t parked_pll_hz = 850000000;
        Si5351Device::Output tx_output = Si5351Device::Output::CLK0;
        bool park_unused_outputs = true;
        bool disable_tx_output_when_idle = true;
    };

    /**
     * @brief Frequency entry for one tone
     */
    struct ToneEntry
    {
        double frequency_hz = 0.0;
    };

    /**
     * @brief Precomputed register set for one tone
     *
     * This should contain only the writes needed to move the active output
     * into the requested tone state, not necessarily a full chip image.
     */
    struct ToneRegisterSet
    {
        double requested_hz = 0.0;
        double actual_hz = 0.0;
        std::vector<Si5351Device::RegisterWrite> writes;
    };

    /**
     * @brief Full planning result for one transmission
     */
    struct Plan
    {
        Mode mode = Mode::TONE;
        std::vector<Si5351Device::RegisterWrite> startup_writes;
        std::vector<Si5351Device::RegisterWrite> idle_writes;
        std::vector<ToneRegisterSet> tone_sets;
    };

    /**
     * @brief Construct a planner
     *
     * @param config Planner configuration
     */
    explicit Si5351Planner(const Config& config);

    /**
     * @brief Build a transmission plan
     *
     * @param mode Transmission mode
     * @param tones Unique tone frequencies to precompute
     * @return Planned startup and tone register data
     */
    Plan buildPlan(
        Mode mode,
        const std::vector<ToneEntry>& tones) const;

    /**
     * @brief Return planner configuration
     *
     * @return Active planner configuration
     */
    const Config& getConfig() const noexcept;

private:
    /**
     * @brief Build startup writes shared by all tones
     *
     * @return Register writes for initial device programming
     */
    std::vector<Si5351Device::RegisterWrite> buildStartupWrites() const;

    /**
     * @brief Build idle writes used when transmission is inactive
     *
     * @return Register writes for the idle state
     */
    std::vector<Si5351Device::RegisterWrite> buildIdleWrites() const;

    /**
     * @brief Build one tone register set
     *
     * @param frequency_hz Requested RF frequency
     * @return Precomputed tone register set
     */
    ToneRegisterSet buildToneRegisterSet(double frequency_hz) const;

    /**
     * @brief Quantize a requested frequency to the achievable output
     *
     * @param requested_hz Requested output frequency
     * @return Achievable output frequency
     */
    double quantizeFrequency(double requested_hz) const;

    Config config_;
};

#endif
