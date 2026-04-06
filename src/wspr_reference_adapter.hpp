/**
 * @file wspr_reference_adapter.hpp
 * @brief Adapter types and helpers for wspr-reference integration.
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

#ifndef WSPR_REFERENCE_ADAPTER_HPP
#define WSPR_REFERENCE_ADAPTER_HPP

#include "wspr_ref_api.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

constexpr std::size_t WSPR_SYMBOL_COUNT = 162U;

struct PreparedWsprFrame
{
    std::array<std::uint8_t, WSPR_SYMBOL_COUNT> symbols{};
};

struct PreparedWsprTransmission
{
    std::string plan_type;
    std::vector<PreparedWsprFrame> frames;
    std::string callsign;
    std::string locator;
    int power_dbm = 0;

    [[nodiscard]] bool empty() const noexcept
    {
        return frames.empty();
    }

    [[nodiscard]] std::size_t frameCount() const noexcept
    {
        return frames.size();
    }

    [[nodiscard]] std::size_t symbolCountPerFrame() const noexcept
    {
        return WSPR_SYMBOL_COUNT;
    }

    [[nodiscard]] std::size_t totalSymbolCount() const noexcept
    {
        return frames.size() * WSPR_SYMBOL_COUNT;
    }
};

PreparedWsprTransmission build_prepared_wspr_transmission(
    const std::string& callsign,
    const std::string& locator,
    int power_dbm);

PreparedWsprTransmission build_prepared_wspr_transmission(
    const std::string& callsign,
    const std::string& locator,
    int power_dbm,
    wspr::TransmissionPlanPreference preference);

PreparedWsprTransmission build_prepared_wspr_transmission(
    const wspr::WsprEncodeResult& result);

#endif // WSPR_REFERENCE_ADAPTER_HPP
