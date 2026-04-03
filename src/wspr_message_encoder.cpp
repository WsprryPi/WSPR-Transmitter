/**
 * @file wspr_message_encoder.cpp
 * @brief WSPR-specific message encoding helper implementation.
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

#include "wspr_message_encoder.hpp"

#include <stdexcept>

#include "wspr_message.hpp"

WsprSymbolSequence encodeWsprMessage(const WsprMessageConfig &config)
{
    switch (config.message_type)
    {
    case WsprMessageType::Type1:
    {
        if (config.call_sign.empty() || config.grid_square.empty() ||
            config.power_dbm == 0)
        {
            throw std::invalid_argument(
                "Type 1 WSPR encoding requires call sign, grid square, and power_dbm.");
        }

        WsprMessage msg(config.call_sign, config.grid_square, config.power_dbm);

        WsprSymbolSequence sequence;
        sequence.symbols.assign(msg.symbols, msg.symbols + msg.size);
        return sequence;
    }

    case WsprMessageType::Type2:
        throw std::runtime_error("WSPR type 2 encoding is not implemented yet.");

    case WsprMessageType::Type3:
        throw std::runtime_error("WSPR type 3 encoding is not implemented yet.");

    default:
        throw std::runtime_error("Unsupported WSPR message type.");
    }
}
