#include "execution_plan_compiler.hpp"

#include <cctype>
#include <stdexcept>
#include <string_view>
#include <variant>

namespace wsprrypi
{
namespace
{

constexpr double kWsprSymbolPeriodSeconds = 8192.0 / 12000.0;
constexpr double kWsprToneSpacingHz = 1.0 / kWsprSymbolPeriodSeconds;

std::string_view morse_code_for(char ch)
{
    switch (std::toupper(static_cast<unsigned char>(ch)))
    {
    case 'A': return ".-";
    case 'B': return "-...";
    case 'C': return "-.-.";
    case 'D': return "-..";
    case 'E': return ".";
    case 'F': return "..-.";
    case 'G': return "--.";
    case 'H': return "....";
    case 'I': return "..";
    case 'J': return ".---";
    case 'K': return "-.-";
    case 'L': return ".-..";
    case 'M': return "--";
    case 'N': return "-.";
    case 'O': return "---";
    case 'P': return ".--.";
    case 'Q': return "--.-";
    case 'R': return ".-.";
    case 'S': return "...";
    case 'T': return "-";
    case 'U': return "..-";
    case 'V': return "...-";
    case 'W': return ".--";
    case 'X': return "-..-";
    case 'Y': return "-.--";
    case 'Z': return "--..";
    case '0': return "-----";
    case '1': return ".----";
    case '2': return "..---";
    case '3': return "...--";
    case '4': return "....-";
    case '5': return ".....";
    case '6': return "-....";
    case '7': return "--...";
    case '8': return "---..";
    case '9': return "----.";
    case '/': return "-..-.";
    case '?': return "..--..";
    case '.': return ".-.-.-";
    case ',': return "--..--";
    case '-': return "-....-";
    case '+': return ".-.-.";
    case '=': return "-...-";
    default: return {};
    }
}

bool is_space_like(char ch)
{
    return std::isspace(static_cast<unsigned char>(ch)) != 0;
}

void validate_positive_duration(
    std::chrono::nanoseconds duration,
    const char* field_name)
{
    if (duration <= std::chrono::nanoseconds::zero())
    {
        throw std::runtime_error(
            std::string("QRSS timing field is invalid: ") + field_name);
    }
}

void append_event(
    ExecutionPlan& plan,
    std::chrono::nanoseconds& offset,
    RfEventType type,
    bool rf_on,
    double frequency_hz,
    std::chrono::nanoseconds duration,
    const EnvelopeSettings& envelope)
{
    if (duration <= std::chrono::nanoseconds::zero())
        return;

    RfEvent event;
    event.offset_from_start = offset;
    event.duration = duration;
    event.type = type;
    event.frequency_hz = frequency_hz;
    event.rf_on = rf_on;
    event.envelope = envelope;
    plan.events.push_back(event);
    offset += duration;
}

std::size_t resolve_wspr_frame_index(const PreparedWsprTransmission& prepared)
{
    if (prepared.frames.empty())
        throw std::runtime_error("WSPR payload has no prepared frames.");

    // PreparedWsprTransmission.current_frame is carried through scheduling as
    // a 1-based "current frame" ordinal for runtime status. Execution needs a
    // zero-based index into frames.
    if (prepared.current_frame == 0U)
        return 0U;

    const std::size_t frame_index = prepared.current_frame - 1U;
    if (frame_index >= prepared.frames.size())
        throw std::runtime_error("WSPR payload current frame index is out of range.");

    return frame_index;
}

double wspr_symbol_frequency(
    double base_frequency_hz,
    double tone_spacing_hz,
    std::uint8_t symbol)
{
    if (symbol > 3U)
        throw std::runtime_error("Invalid WSPR symbol.");

    return base_frequency_hz
        - 1.5 * tone_spacing_hz
        + static_cast<double>(symbol) * tone_spacing_hz;
}

PlanSummary build_summary(const std::vector<RfEvent>& events)
{
    PlanSummary summary{};

    summary.event_count = events.size();
    if (events.empty())
        return summary;

    summary.total_duration =
        events.back().offset_from_start + events.back().duration;

    bool first = true;
    for (const auto& event : events)
    {
        if (!event.rf_on)
            continue;

        if (first)
        {
            summary.min_frequency_hz = event.frequency_hz;
            summary.max_frequency_hz = event.frequency_hz;
            first = false;
        }
        else
        {
            if (event.frequency_hz < summary.min_frequency_hz)
                summary.min_frequency_hz = event.frequency_hz;
            if (event.frequency_hz > summary.max_frequency_hz)
                summary.max_frequency_hz = event.frequency_hz;
        }
    }

    return summary;
}

} // namespace

ExecutionPlan ExecutionPlanCompiler::compile(
    const TransmissionRequest& request) const
{
    return std::visit(
        [this, &request](const auto& payload) -> ExecutionPlan
        {
            using PayloadT = std::decay_t<decltype(payload)>;

            if constexpr (std::is_same_v<PayloadT, WsprPayload>)
                return compile_wspr(request, payload);
            else if constexpr (std::is_same_v<PayloadT, QrssPayload>)
                return compile_qrss(request, payload);
            else if constexpr (std::is_same_v<PayloadT, FskcwPayload>)
                return compile_fskcw(request, payload);
            else if constexpr (std::is_same_v<PayloadT, DfcwPayload>)
                return compile_dfcw(request, payload);
            else if constexpr (std::is_same_v<PayloadT, CwPayload>)
                return compile_cw(request, payload);
            else
                return compile_tone(request, payload);
        },
        request.payload);
}

ExecutionPlan ExecutionPlanCompiler::compile_wspr(
    const TransmissionRequest& request,
    const WsprPayload& payload) const
{
    const std::size_t frame_index = resolve_wspr_frame_index(payload.prepared);

    if (payload.base_frequency_hz <= 0.0)
        throw std::runtime_error("WSPR payload base frequency is invalid.");

    const auto symbol_duration =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(kWsprSymbolPeriodSeconds));

    const auto& frame = payload.prepared.frames[frame_index];

    ExecutionPlan plan;
    plan.request_id = request.id;
    plan.mode = request.mode;
    plan.backend = request.output.backend;
    plan.reference_frequency_hz = payload.base_frequency_hz;
    plan.calibration = request.calibration;
    plan.policy = request.policy;
    plan.events.reserve(frame.symbols.size());

    std::chrono::nanoseconds offset{0};

    for (std::uint8_t symbol : frame.symbols)
    {
        RfEvent event;
        event.offset_from_start = offset;
        event.duration = symbol_duration;
        event.type = RfEventType::HOLD;
        event.frequency_hz = wspr_symbol_frequency(
            payload.base_frequency_hz,
            kWsprToneSpacingHz,
            symbol);
        event.rf_on = true;
        event.envelope = payload.envelope;

        plan.events.push_back(event);
        offset += symbol_duration;
    }

    plan.summary = build_summary(plan.events);
    return plan;
}

ExecutionPlan ExecutionPlanCompiler::compile_qrss(
    const TransmissionRequest& request,
    const QrssPayload& payload) const
{
    if (payload.message.empty())
        throw std::runtime_error("QRSS payload message is empty.");

    if (payload.frequency_hz <= 0.0)
        throw std::runtime_error("QRSS payload frequency is invalid.");

    validate_positive_duration(payload.timing.dot, "dot");
    validate_positive_duration(payload.timing.dash, "dash");
    validate_positive_duration(payload.timing.intra_element_gap, "intra_element_gap");
    validate_positive_duration(payload.timing.inter_character_gap, "inter_character_gap");
    validate_positive_duration(payload.timing.inter_word_gap, "inter_word_gap");

    ExecutionPlan plan;
    plan.request_id = request.id;
    plan.mode = request.mode;
    plan.backend = request.output.backend;
    // The current Raspberry Pi backend compatibility path still configures a
    // WSPR-style 4-tone table. QRSS execution uses symbol 0 from that table,
    // so the backend reference stays offset by 1.5 tone spacings while the
    // emitted events carry the fixed user-requested frequency.
    plan.reference_frequency_hz = payload.frequency_hz + 1.5 * kWsprToneSpacingHz;
    plan.calibration = request.calibration;
    plan.policy = request.policy;

    std::chrono::nanoseconds offset{0};

    for (std::size_t i = 0; i < payload.message.size(); ++i)
    {
        const char ch = payload.message[i];

        if (is_space_like(ch))
        {
            continue;
        }

        const std::string_view morse = morse_code_for(ch);
        if (morse.empty())
        {
            throw std::runtime_error("QRSS payload contains unsupported character.");
        }

        for (std::size_t j = 0; j < morse.size(); ++j)
        {
            const auto duration =
                (morse[j] == '.') ? payload.timing.dot : payload.timing.dash;

            append_event(
                plan,
                offset,
                RfEventType::RF_ON,
                true,
                payload.frequency_hz,
                duration,
                payload.envelope);

            if (j + 1U < morse.size())
            {
                append_event(
                    plan,
                    offset,
                    RfEventType::RF_OFF,
                    false,
                    payload.frequency_hz,
                    payload.timing.intra_element_gap,
                    payload.envelope);
            }
        }

        std::size_t next = i + 1U;
        while (next < payload.message.size() && is_space_like(payload.message[next]))
            ++next;

        if (next >= payload.message.size())
            continue;

        const auto gap =
            (next > (i + 1U)) ? payload.timing.inter_word_gap
                              : payload.timing.inter_character_gap;
        append_event(
            plan,
            offset,
            RfEventType::RF_OFF,
            false,
            payload.frequency_hz,
            gap,
            payload.envelope);
    }

    if (plan.events.empty())
        throw std::runtime_error("QRSS payload produced no execution events.");

    plan.summary = build_summary(plan.events);
    return plan;
}

ExecutionPlan ExecutionPlanCompiler::compile_fskcw(
    const TransmissionRequest&,
    const FskcwPayload&) const
{
    throw std::runtime_error("FSKCW execution-plan compilation is not implemented.");
}

ExecutionPlan ExecutionPlanCompiler::compile_dfcw(
    const TransmissionRequest&,
    const DfcwPayload&) const
{
    throw std::runtime_error("DFCW execution-plan compilation is not implemented.");
}

ExecutionPlan ExecutionPlanCompiler::compile_cw(
    const TransmissionRequest&,
    const CwPayload&) const
{
    throw std::runtime_error("CW execution-plan compilation is not implemented.");
}

ExecutionPlan ExecutionPlanCompiler::compile_tone(
    const TransmissionRequest&,
    const TonePayload&) const
{
    throw std::runtime_error("Tone execution-plan compilation is not implemented.");
}

} // namespace wsprrypi
