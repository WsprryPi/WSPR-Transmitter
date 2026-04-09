#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "execution_plan.hpp"

namespace wsprrypi
{

struct BackendInfo
{
    BackendKind kind{BackendKind::RPI_CLOCK_GPIO};
    std::string name;
    std::string description;
};

struct BackendCapabilities
{
    bool supports_frequency_switching{true};
    bool supports_rf_gating{true};
    bool supports_fade_shape{false};
    bool supports_continuous_phase{false};
    bool supports_precomputed_execution{false};

    std::chrono::nanoseconds min_event_duration{};
    double min_frequency_hz{0.0};
    double max_frequency_hz{0.0};
    double nominal_frequency_resolution_hz{0.0};
};

struct BackendAdjustment
{
    std::size_t event_index{0};
    double requested_frequency_hz{0.0};
    double actual_frequency_hz{0.0};
    std::string note;
};

struct BackendCompileResult
{
    bool ok{false};
    std::vector<BackendAdjustment> adjustments{};
    std::string error;
};

struct BackendExecutionInputs
{
    int power_level{0};
    int tx_gpio{0};
};

struct ExecutionResult
{
    bool ok{false};
    bool stopped{false};
    bool faulted{false};
    std::string error;
};

class ITransmissionBackend
{
public:
    virtual ~ITransmissionBackend() = default;

    virtual BackendInfo info() const = 0;
    virtual BackendCapabilities capabilities() const = 0;

    virtual BackendCompileResult configure(
        const ExecutionPlan& plan,
        const BackendExecutionInputs& inputs) = 0;
    virtual ExecutionResult execute(const ExecutionPlan& plan) = 0;

    virtual void stop() noexcept = 0;
    virtual void cleanup() noexcept = 0;
};

} // namespace wsprrypi
