#include "si5351_planner.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace
{
    constexpr std::uint8_t kOutputEnableRegister = 3;
    constexpr std::uint8_t kClkControlBaseRegister = 16;
    constexpr std::uint8_t kPllResetRegister = 177;
    constexpr std::uint8_t kMs0ParameterBaseRegister = 42;
    constexpr std::uint8_t kIntegerMode = 0x40;
    constexpr std::uint8_t kMultisynthSource = 0x0c;
    constexpr std::uint8_t kDivideBy4 = 0x0c;
    constexpr std::uint32_t kMaxDenominator = 1048575;

    int failures = 0;

    void expect(bool condition, const std::string& message)
    {
        if (condition)
            return;

        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }

    bool register_value(
        const std::vector<Si5351Device::RegisterWrite>& writes,
        std::uint8_t address,
        std::uint8_t& value)
    {
        for (const Si5351Device::RegisterWrite& write : writes)
        {
            if (write.address != address)
                continue;

            value = write.value;
            return true;
        }

        return false;
    }

    Si5351Planner::Plan build_plan(
        std::uint64_t parked_pll_hz,
        double multisynth_ratio,
        Si5351Device::Output output = Si5351Device::Output::CLK0)
    {
        Si5351Planner::Config config;
        config.reference_hz = 27000000;
        config.parked_pll_hz = parked_pll_hz;
        config.tx_output = output;

        const double requested_hz =
            static_cast<double>(parked_pll_hz) / multisynth_ratio;
        return Si5351Planner(config).buildPlan(
            Si5351Planner::Mode::TONE,
            {Si5351Planner::ToneEntry{requested_hz}});
    }

    bool valid_tone(const Si5351Planner::Plan& plan)
    {
        return plan.tone_sets.size() == 1 &&
            plan.tone_sets.front().actual_hz > 0.0 &&
            !plan.tone_sets.front().writes.empty();
    }

    void expect_valid_ratio(double ratio, const std::string& label)
    {
        const Si5351Planner::Plan plan = build_plan(600000000, ratio);
        expect(valid_tone(plan), label + " should be accepted");
    }

    void expect_invalid_ratio(double ratio, const std::string& label)
    {
        const Si5351Planner::Plan plan = build_plan(600000000, ratio);
        expect(!valid_tone(plan), label + " should be rejected");
        if (plan.tone_sets.size() == 1)
        {
            expect(
                plan.tone_sets.front().actual_hz == 0.0,
                label + " should report zero achievable frequency");
            expect(
                plan.tone_sets.front().writes.empty(),
                label + " should produce no tone writes");
        }
    }

    void test_documented_ratio_domain()
    {
        expect_valid_ratio(4.0, "exact divide-by-4");
        expect_valid_ratio(6.0, "exact divide-by-6");
        expect_valid_ratio(8.0, "exact divide-by-8");

        expect_invalid_ratio(3.999, "ratio below divide-by-4");
        expect_invalid_ratio(4.001, "fractional ratio above 4");
        expect_invalid_ratio(5.999, "fractional ratio below 6");
        expect_invalid_ratio(6.1, "fractional ratio above 6");
        expect_invalid_ratio(7.9, "fractional ratio below 8");

        const double minimum_fractional = 8.0 +
            1.0 / static_cast<double>(kMaxDenominator);
        expect_invalid_ratio(
            8.0 + 0.5 / static_cast<double>(kMaxDenominator),
            "fractional ratio below documented minimum");
        expect_valid_ratio(
            minimum_fractional,
            "documented minimum fractional ratio");
        expect_valid_ratio(9.5, "representative fractional ratio");
        expect_valid_ratio(2048.0, "maximum ratio");
        expect_invalid_ratio(2048.001, "ratio above maximum");
    }

    void test_special_integer_encoding()
    {
        const Si5351Device::Output outputs[] = {
            Si5351Device::Output::CLK0,
            Si5351Device::Output::CLK1,
            Si5351Device::Output::CLK2};

        for (std::uint8_t index = 0; index < 3; ++index)
        {
            const Si5351Planner::Plan plan =
                build_plan(600000000, 4.0, outputs[index]);
            expect(valid_tone(plan), "divide-by-4 should work on CLK0-2");
            if (!valid_tone(plan))
                continue;

            const auto& writes = plan.tone_sets.front().writes;
            const std::uint8_t base = static_cast<std::uint8_t>(
                kMs0ParameterBaseRegister + index * 8);
            std::uint8_t value = 0;
            expect(
                register_value(writes, base, value) && value == 0,
                "divide-by-4 P3 high byte should be zero");
            expect(
                register_value(writes, base + 1, value) && value == 1,
                "divide-by-4 P3 low byte should be one");
            expect(
                register_value(writes, base + 2, value) &&
                    (value & kDivideBy4) == kDivideBy4 &&
                    (value & 0x03) == 0,
                "divide-by-4 control bits and P1 high bits should be exact");
            expect(
                register_value(writes, base + 3, value) && value == 0,
                "divide-by-4 P1 middle byte should be zero");
            expect(
                register_value(writes, base + 4, value) && value == 0,
                "divide-by-4 P1 low byte should be zero");
            expect(
                register_value(writes, base + 5, value) && value == 0,
                "divide-by-4 P2/P3 high byte should be zero");
            expect(
                register_value(writes, base + 6, value) && value == 0,
                "divide-by-4 P2 middle byte should be zero");
            expect(
                register_value(writes, base + 7, value) && value == 0,
                "divide-by-4 P2 low byte should be zero");
            expect(
                register_value(
                    writes,
                    static_cast<std::uint8_t>(
                        kClkControlBaseRegister + index),
                    value) &&
                    value == (kIntegerMode | kMultisynthSource),
                "divide-by-4 should select integer MultiSynth mode");
        }

        for (const double ratio : {6.0, 8.0, 10.0})
        {
            const Si5351Planner::Plan plan = build_plan(600000000, ratio);
            std::uint8_t value = 0;
            expect(
                valid_tone(plan) &&
                    register_value(
                        plan.tone_sets.front().writes,
                        kClkControlBaseRegister,
                        value) &&
                    value == (kIntegerMode | kMultisynthSource),
                "even integer divider should select integer mode");
        }

        const Si5351Planner::Plan odd_integer = build_plan(600000000, 9.0);
        std::uint8_t odd_control = 0;
        expect(
            valid_tone(odd_integer) &&
                register_value(
                    odd_integer.tone_sets.front().writes,
                    kClkControlBaseRegister,
                    odd_control) &&
                odd_control == kMultisynthSource,
            "odd integer divider should remain in fractional mode");
    }

    void test_pll_frequency_domain()
    {
        for (const std::uint64_t pll_hz : {600000000ULL, 900000000ULL})
        {
            const Si5351Planner::Plan plan = build_plan(pll_hz, 9.0);
            std::uint8_t reset = 0;
            expect(valid_tone(plan), "PLL boundary should allow a tone");
            expect(
                register_value(plan.startup_writes, kPllResetRegister, reset),
                "PLL boundary should produce startup PLL programming");
        }

        for (const std::uint64_t pll_hz : {599999999ULL, 900000001ULL})
        {
            const Si5351Planner::Plan plan = build_plan(pll_hz, 9.0);
            std::uint8_t reset = 0;
            expect(!valid_tone(plan), "out-of-range PLL should reject tone");
            expect(
                !register_value(
                    plan.startup_writes,
                    kPllResetRegister,
                    reset),
                "out-of-range PLL should not be programmed or reset");
            std::uint8_t disabled = 0;
            expect(
                register_value(
                    plan.startup_writes,
                    kOutputEnableRegister,
                    disabled) &&
                    disabled == 0xff,
                "out-of-range PLL plan should begin with all outputs disabled");
        }
    }

    void test_rejection_remains_output_disabled()
    {
        const Si5351Planner::Plan plan = build_plan(600000000, 6.1);
        expect(!valid_tone(plan), "invalid divider should reject the tone");

        std::uint8_t output_enable = 0;
        expect(
            register_value(
                plan.startup_writes,
                kOutputEnableRegister,
                output_enable) &&
                output_enable == 0xff,
            "rejected plan should keep every output disabled");

        for (std::uint8_t index = 0; index < 3; ++index)
        {
            std::uint8_t control = 0;
            expect(
                register_value(
                    plan.startup_writes,
                    static_cast<std::uint8_t>(
                        kClkControlBaseRegister + index),
                    control) &&
                    (control & 0x80) != 0,
                "rejected plan should keep each output driver powered down");
        }
    }

    void test_representative_existing_frequencies()
    {
        constexpr double frequencies_hz[] = {
            474200.0,
            1836600.0,
            3568600.0,
            7038600.0,
            10138700.0,
            14095600.0,
            18104600.0,
            21094600.0,
            24924600.0,
            28124600.0,
            50293000.0};

        Si5351Planner::Config config;
        for (const double frequency_hz : frequencies_hz)
        {
            const Si5351Planner::Plan plan = Si5351Planner(config).buildPlan(
                Si5351Planner::Mode::TONE,
                {Si5351Planner::ToneEntry{frequency_hz}});
            expect(
                valid_tone(plan),
                "representative existing frequency " +
                    std::to_string(frequency_hz) +
                    " Hz should remain plannable");
            if (!valid_tone(plan))
                continue;

            const double error_hz = std::fabs(
                plan.tone_sets.front().actual_hz - frequency_hz);
            expect(
                error_hz < 5.0,
                "representative existing frequency " +
                    std::to_string(frequency_hz) +
                    " Hz error should remain below the existing "
                    "fixed-denominator resolution bound");
        }
    }
}

int main()
{
    test_documented_ratio_domain();
    test_special_integer_encoding();
    test_pll_frequency_domain();
    test_rejection_remains_output_disabled();
    test_representative_existing_frequencies();

    if (failures != 0)
    {
        std::cerr << failures << " Si5351 planner test(s) failed.\n";
        return 1;
    }

    std::cout << "Si5351 planner tests passed.\n";
    return 0;
}
