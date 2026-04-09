#include "execution_plan_compiler.hpp"

#include <stdexcept>
#include <variant>

namespace wsprrypi
{
namespace
{

constexpr double kWsprSymbolPeriodSeconds = 8192.0 / 12000.0;
constexpr double kWsprToneSpacingHz = 1.0 / kWsprSymbolPeriodSeconds;

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
    if (payload.prepared.frames.empty())
        throw std::runtime_error("WSPR payload has no prepared frames.");

    if (payload.prepared.current_frame >= payload.prepared.frames.size())
        throw std::runtime_error("WSPR payload current frame index is out of range.");

    if (payload.base_frequency_hz <= 0.0)
        throw std::runtime_error("WSPR payload base frequency is invalid.");

    const auto symbol_duration =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(kWsprSymbolPeriodSeconds));

    const auto& frame = payload.prepared.frames[payload.prepared.current_frame];

    ExecutionPlan plan;
    plan.request_id = request.id;
    plan.mode = request.mode;
    plan.backend = request.output.backend;
    plan.reference_frequency_hz = payload.base_frequency_hz;
    plan.tx_gpio = request.output.gpio;
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
    const TransmissionRequest&,
    const QrssPayload&) const
{
    throw std::runtime_error("QRSS execution-plan compilation is not implemented.");
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
