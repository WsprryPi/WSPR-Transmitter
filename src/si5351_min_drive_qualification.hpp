#pragma once

#include "si5351_device.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace si5351_min_drive_qualification
{
constexpr double frequency_hz = 144490497.802734375;
constexpr unsigned duration_ms = 600000;

struct Result
{
    bool ok{false};
    std::string error;
    double actual_hz{0.0};
    std::uint8_t before{0};
    std::uint8_t after_inhibit{0};
    std::uint8_t after_cleanup{0};
    std::vector<std::pair<std::uint8_t, std::uint8_t>> writes;
};

bool parse_acknowledgement(int argc, char** argv, std::string& error);
Result run(
    std::shared_ptr<Si5351Device::I2CAdapter> adapter,
    const std::function<bool(unsigned)>& wait_ms);
std::shared_ptr<Si5351Device::I2CAdapter> make_system_adapter();
bool system_wait_ms(unsigned milliseconds);
void request_stop() noexcept;
void clear_stop() noexcept;
} // namespace si5351_min_drive_qualification
