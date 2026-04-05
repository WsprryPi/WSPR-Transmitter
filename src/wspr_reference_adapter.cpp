/**
 * @file wspr_reference_adapter.cpp
 * @brief Adapter helpers for wspr-reference integration.
 */

#include "wspr_reference_adapter.hpp"

#include <sstream>
#include <stdexcept>

namespace
{
PreparedWsprFrame to_frame(const std::string& symbols)
{
    if (symbols.size() != WSPR_SYMBOL_COUNT)
    {
        std::ostringstream oss;
        oss << "Encoded WSPR frame has invalid length: " << symbols.size();
        throw std::runtime_error(oss.str());
    }

    PreparedWsprFrame frame;
    for (std::size_t i = 0; i < WSPR_SYMBOL_COUNT; ++i)
    {
        const char ch = symbols[i];
        if (ch < '0' || ch > '3')
        {
            std::ostringstream oss;
            oss << "Encoded WSPR frame contains invalid symbol at index "
                << i << ".";
            throw std::runtime_error(oss.str());
        }
        frame.symbols[i] = static_cast<std::uint8_t>(ch - '0');
    }

    return frame;
}
} // namespace

PreparedWsprTransmission build_prepared_wspr_transmission(
    const std::string& callsign,
    const std::string& locator,
    int power_dbm)
{
    return build_prepared_wspr_transmission(
        wspr::encode_message(callsign, locator, power_dbm));
}

PreparedWsprTransmission build_prepared_wspr_transmission(
    const wspr::WsprEncodeResult& result)
{
    if (!result.ok)
    {
        throw std::runtime_error(result.error.empty() ?
            "WSPR encoding failed." : result.error);
    }

    PreparedWsprTransmission plan;
    plan.plan_type = result.type;
    plan.callsign = result.callsign;
    plan.locator = result.locator;
    plan.power_dbm = result.power_dbm;

    if (!result.symbols_list.empty())
    {
        for (const auto& symbols : result.symbols_list)
            plan.frames.push_back(to_frame(symbols));
    }
    else if (!result.symbols.empty())
    {
        plan.frames.push_back(to_frame(result.symbols));
    }

    if (plan.frames.empty())
        throw std::runtime_error("WSPR encoding returned no frames.");

    return plan;
}
