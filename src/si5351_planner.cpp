#include "si5351_planner.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

namespace
{
    static constexpr std::uint8_t kOutputEnableRegister = 3;
    static constexpr std::uint8_t kClkControlBaseRegister = 16;
    static constexpr std::uint8_t kPllAParameterBaseRegister = 26;
    static constexpr std::uint8_t kMs0ParameterBaseRegister = 42;
    static constexpr std::uint8_t kPllResetRegister = 177;
    static constexpr std::uint8_t kOutputDisableAll = 0xff;
    static constexpr std::uint8_t kClkPowerDown = 0x80;
    static constexpr std::uint8_t kClkInputMultisynth = 0x0c;
    static constexpr std::uint8_t kResetPllA = 0x20;
    static constexpr std::uint32_t kMaxFractionDenominator = 1048575;
    static constexpr std::uint32_t kMinPllMultiplier = 15;
    static constexpr std::uint32_t kMaxPllMultiplier = 90;
    static constexpr std::uint32_t kMinMultisynthDivider = 6;
    static constexpr std::uint32_t kMaxMultisynthDivider = 1800;

    struct DividerParameters
    {
        bool valid = false;
        std::uint32_t a = 0;
        std::uint32_t b = 0;
        std::uint32_t c = 1;
        std::uint32_t p1 = 0;
        std::uint32_t p2 = 0;
        std::uint32_t p3 = 1;
        double actual_ratio = 0.0;
    };

    static bool output_index(
        Si5351Device::Output output,
        std::uint8_t& index)
    {
        switch (output)
        {
            case Si5351Device::Output::CLK0:
                index = 0;
                return true;
            case Si5351Device::Output::CLK1:
                index = 1;
                return true;
            case Si5351Device::Output::CLK2:
                index = 2;
                return true;
        }

        return false;
    }

    static DividerParameters build_divider_parameters(
        double ratio,
        std::uint32_t minimum_integer,
        std::uint32_t maximum_integer)
    {
        DividerParameters params;
        if (!std::isfinite(ratio) || ratio <= 0.0)
            return params;

        const double integer_part = std::floor(ratio);
        if (integer_part < static_cast<double>(minimum_integer) ||
            integer_part > static_cast<double>(maximum_integer))
        {
            return params;
        }

        params.a = static_cast<std::uint32_t>(integer_part);
        params.c = kMaxFractionDenominator;

        const double fractional_part =
            ratio - static_cast<double>(params.a);
        params.b = static_cast<std::uint32_t>(
            std::llround(fractional_part * params.c));

        if (params.b >= params.c)
        {
            params.a += 1;
            params.b = 0;
        }

        if (params.a < minimum_integer || params.a > maximum_integer)
            return params;

        const std::uint32_t intermediate =
            static_cast<std::uint32_t>(
                (128ULL * params.b) / params.c);

        params.p1 = 128U * params.a + intermediate - 512U;
        params.p2 = 128U * params.b - params.c * intermediate;
        params.p3 = params.c;
        params.actual_ratio = static_cast<double>(params.a) +
            static_cast<double>(params.b) / static_cast<double>(params.c);
        params.valid = true;
        return params;
    }

    static void append_parameter_writes(
        std::vector<Si5351Device::RegisterWrite>& writes,
        std::uint8_t base_register,
        const DividerParameters& params)
    {
        writes.push_back(Si5351Device::RegisterWrite{
            base_register,
            static_cast<std::uint8_t>((params.p3 >> 8) & 0xff)});
        writes.push_back(Si5351Device::RegisterWrite{
            static_cast<std::uint8_t>(base_register + 1),
            static_cast<std::uint8_t>(params.p3 & 0xff)});
        writes.push_back(Si5351Device::RegisterWrite{
            static_cast<std::uint8_t>(base_register + 2),
            static_cast<std::uint8_t>((params.p1 >> 16) & 0x03)});
        writes.push_back(Si5351Device::RegisterWrite{
            static_cast<std::uint8_t>(base_register + 3),
            static_cast<std::uint8_t>((params.p1 >> 8) & 0xff)});
        writes.push_back(Si5351Device::RegisterWrite{
            static_cast<std::uint8_t>(base_register + 4),
            static_cast<std::uint8_t>(params.p1 & 0xff)});
        writes.push_back(Si5351Device::RegisterWrite{
            static_cast<std::uint8_t>(base_register + 5),
            static_cast<std::uint8_t>(
                (((params.p3 >> 16) & 0x0f) << 4) |
                ((params.p2 >> 16) & 0x0f))});
        writes.push_back(Si5351Device::RegisterWrite{
            static_cast<std::uint8_t>(base_register + 6),
            static_cast<std::uint8_t>((params.p2 >> 8) & 0xff)});
        writes.push_back(Si5351Device::RegisterWrite{
            static_cast<std::uint8_t>(base_register + 7),
            static_cast<std::uint8_t>(params.p2 & 0xff)});
    }

    static std::uint8_t multisynth_base_register(std::uint8_t index)
    {
        return static_cast<std::uint8_t>(
            kMs0ParameterBaseRegister + index * 8);
    }

    static std::uint8_t clock_control_register(std::uint8_t index)
    {
        return static_cast<std::uint8_t>(
            kClkControlBaseRegister + index);
    }
}

/**
 * @brief Construct a planner
 *
 * @param config Planner configuration
 */
Si5351Planner::Si5351Planner(const Config& config)
    : config_(config)
{
}

Si5351Planner::Plan Si5351Planner::buildPlan(
    Mode mode,
    const std::vector<ToneEntry>& tones) const
{
    Plan plan;
    plan.mode = mode;
    plan.startup_writes = buildStartupWrites();
    plan.idle_writes = buildIdleWrites();
    plan.tone_sets.reserve(tones.size());

    for (const ToneEntry& tone : tones)
    {
        plan.tone_sets.push_back(
            buildToneRegisterSet(tone.frequency_hz));
    }

    return plan;
}

const Si5351Planner::Config& Si5351Planner::getConfig() const noexcept
{
    return config_;
}

std::vector<Si5351Device::RegisterWrite>
Si5351Planner::buildStartupWrites() const
{
    std::vector<Si5351Device::RegisterWrite> writes;

    writes.push_back(Si5351Device::RegisterWrite{
        kOutputEnableRegister,
        kOutputDisableAll});

    std::uint8_t tx_index = 0;
    if (!output_index(config_.tx_output, tx_index))
        return writes;

    const std::uint8_t powered_down_control =
        static_cast<std::uint8_t>(kClkPowerDown | kClkInputMultisynth);

    writes.push_back(Si5351Device::RegisterWrite{
        clock_control_register(tx_index),
        powered_down_control});

    if (config_.park_unused_outputs)
    {
        for (std::uint8_t index = 0; index < 3; ++index)
        {
            if (index == tx_index)
                continue;

            writes.push_back(Si5351Device::RegisterWrite{
                clock_control_register(index),
                powered_down_control});
        }
    }

    if (config_.reference_hz == 0 || config_.parked_pll_hz == 0)
        return writes;

    const double pll_ratio =
        static_cast<double>(config_.parked_pll_hz) /
        static_cast<double>(config_.reference_hz);
    const DividerParameters pll_params = build_divider_parameters(
        pll_ratio,
        kMinPllMultiplier,
        kMaxPllMultiplier);

    if (!pll_params.valid)
        return writes;

    append_parameter_writes(
        writes,
        kPllAParameterBaseRegister,
        pll_params);
    writes.push_back(Si5351Device::RegisterWrite{
        kPllResetRegister,
        kResetPllA});

    return writes;
}

std::vector<Si5351Device::RegisterWrite>
Si5351Planner::buildIdleWrites() const
{
    std::vector<Si5351Device::RegisterWrite> writes;
    if (!config_.disable_tx_output_when_idle)
        return writes;

    std::uint8_t tx_index = 0;
    if (!output_index(config_.tx_output, tx_index))
        return writes;

    const std::uint8_t powered_down_control =
        static_cast<std::uint8_t>(kClkPowerDown | kClkInputMultisynth);

    writes.push_back(Si5351Device::RegisterWrite{
        kOutputEnableRegister,
        kOutputDisableAll});
    writes.push_back(Si5351Device::RegisterWrite{
        clock_control_register(tx_index),
        powered_down_control});

    return writes;
}

Si5351Planner::ToneRegisterSet
Si5351Planner::buildToneRegisterSet(double frequency_hz) const
{
    ToneRegisterSet tone;
    tone.requested_hz = frequency_hz;
    tone.actual_hz = quantizeFrequency(frequency_hz);

    std::uint8_t tx_index = 0;
    if (!output_index(config_.tx_output, tx_index))
        return tone;

    if (frequency_hz <= 0.0 || config_.parked_pll_hz == 0)
        return tone;

    const double multisynth_ratio =
        static_cast<double>(config_.parked_pll_hz) / frequency_hz;
    const DividerParameters ms_params = build_divider_parameters(
        multisynth_ratio,
        kMinMultisynthDivider,
        kMaxMultisynthDivider);

    if (!ms_params.valid)
    {
        tone.actual_hz = 0.0;
        return tone;
    }

    append_parameter_writes(
        tone.writes,
        multisynth_base_register(tx_index),
        ms_params);
    tone.writes.push_back(Si5351Device::RegisterWrite{
        clock_control_register(tx_index),
        kClkInputMultisynth});

    return tone;
}

double Si5351Planner::quantizeFrequency(double requested_hz) const
{
    if (requested_hz <= 0.0 || config_.parked_pll_hz == 0)
        return 0.0;

    const double multisynth_ratio =
        static_cast<double>(config_.parked_pll_hz) / requested_hz;
    const DividerParameters ms_params = build_divider_parameters(
        multisynth_ratio,
        kMinMultisynthDivider,
        kMaxMultisynthDivider);

    if (!ms_params.valid || ms_params.actual_ratio <= 0.0)
        return 0.0;

    return static_cast<double>(config_.parked_pll_hz) /
        ms_params.actual_ratio;
}
