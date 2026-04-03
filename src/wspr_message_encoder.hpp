/**
 * @file wspr_message_encoder.hpp
 * @brief WSPR-specific message encoding helper interface.
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

#ifndef WSPR_MESSAGE_ENCODER_HPP
#define WSPR_MESSAGE_ENCODER_HPP

#include "wspr_transmit_types.hpp"

/**
 * @brief Encode a WSPR message configuration into a transmit-ready symbol
 *        sequence.
 *
 * @param config Input-level WSPR message description.
 * @return Encoded WSPR symbol sequence.
 *
 * @throws std::invalid_argument if the message configuration is incomplete.
 * @throws std::runtime_error if the message type is not yet supported.
 */
WsprSymbolSequence encodeWsprMessage(const WsprMessageConfig &config);

#endif
