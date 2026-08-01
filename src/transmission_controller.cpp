#include "transmission_controller.hpp"

namespace wsprrypi
{

TransmissionController::TransmissionController(
    IExecutionPlanCompiler& compiler,
    ITransmissionBackend& backend)
    : compiler_(compiler),
      backend_(backend)
{
}

BackendCompileResult TransmissionController::prepare(
    const TransmissionRequest& request,
    const TransmissionPrepareOptions& options)
{
    prepared_plan_ = compiler_.compile(request);
    const BackendCompileResult configure_result = backend_.configure(
        *prepared_plan_,
        build_backend_inputs(request, options));
    if (!configure_result.ok)
    {
        prepared_plan_.reset();
        return configure_result;
    }

    apply_adjustments(configure_result);
    return configure_result;
}

ExecutionResult TransmissionController::execute_prepared()
{
    if (!prepared_plan_.has_value())
    {
        return ExecutionResult{
            false,
            false,
            false,
            "No prepared execution plan."};
    }

    return backend_.execute(*prepared_plan_);
}

ExecutionResult TransmissionController::transmit(
    const TransmissionRequest& request,
    const TransmissionPrepareOptions& options)
{
    const BackendCompileResult configure_result = prepare(request, options);
    if (!configure_result.ok)
    {
        return ExecutionResult{
            false,
            false,
            false,
            configure_result.error};
    }

    return execute_prepared();
}

const ExecutionPlan* TransmissionController::prepared_plan() const noexcept
{
    return prepared_plan_.has_value() ? &*prepared_plan_ : nullptr;
}

void TransmissionController::reset() noexcept
{
    prepared_plan_.reset();
}

void TransmissionController::apply_adjustments(
    const BackendCompileResult& configure_result)
{
    if (!prepared_plan_.has_value() || configure_result.adjustments.empty())
        return;

    const auto& adjustment = configure_result.adjustments.front();
    const double delta_hz =
        adjustment.actual_frequency_hz - adjustment.requested_frequency_hz;
    if (delta_hz == 0.0)
        return;

    prepared_plan_->reference_frequency_hz += delta_hz;
    for (auto& event : prepared_plan_->events)
    {
        if (event.rf_on ||
            prepared_plan_->mode == TransmissionMode::STANDARD_FELD)
            event.frequency_hz += delta_hz;
    }
    prepared_plan_->summary.min_frequency_hz += delta_hz;
    prepared_plan_->summary.max_frequency_hz += delta_hz;
}

BackendExecutionInputs TransmissionController::build_backend_inputs(
    const TransmissionRequest& request,
    const TransmissionPrepareOptions& options) const noexcept
{
    BackendExecutionInputs inputs;
    inputs.power_level = options.power_level;
    inputs.tx_gpio = request.output.gpio;
    return inputs;
}

void TransmissionController::stop() noexcept
{
    backend_.stop();
}

} // namespace wsprrypi
