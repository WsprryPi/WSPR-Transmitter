#include "rp1_gpclk_transmit_backend.hpp"
#include "wspr_transmit.hpp"

#include <iostream>
#include <memory>
#include <string>

namespace
{
int failures;
void expect(bool value, const char* message)
{
    if (!value) { std::cerr << "FAIL: " << message << '\n'; ++failures; }
}

class Owner final : public IControllerBridge
{
public:
    WsprTransmitState backendStateValue() const noexcept override { return WsprTransmitState::ENABLED; }
    void backendSetStateValue(WsprTransmitState) noexcept override {}
    bool backendShouldStop() const noexcept override { return stop; }
    void backendSignalStopRequest() noexcept override { stop = true; }
    void backendRequestStopTxNoJoin() noexcept override { stop = true; }
    bool backendWaitInterruptableFor(std::chrono::nanoseconds) override { return !stop; }
    void backendThrowIfStopRequested(const char*) override {}
    void backendReportExecutionProgress(std::size_t) noexcept override {}
    void backendFireTransmitCallback(WsprTransmissionCallbackEvent, WsprTransmitLogLevel, const std::string&, double) override {}
    bool backendRestartCurrentConfiguration() override { return false; }
    bool stop{false};
};

class Provider final : public wsprrypi::Rp1GpclkProvider
{
public:
    bool acquire(std::uint32_t drive, std::string& error) override {
        if (fail_acquire) { error=acquire_error; return false; }
        drive_ma=drive; acquired=true; return true;
    }
    bool submit(const wsprrypi::Rp1GpclkProviderProgram& value, std::string& error) override {
        if (fail_submit) { error=submit_error; return false; }
        program=value; submitted=true; state_value=wsprrypi::Rp1GpclkCompletionState::complete; return true;
    }
    bool requestFiniteStop(std::uint64_t, std::string&) override { stopped=true; return true; }
    wsprrypi::Rp1GpclkCompletionState state(std::uint64_t) const noexcept override { return state_value; }
    void release() noexcept override { released=true; }
    std::uint32_t drive_ma{}; bool acquired{},submitted{},stopped{},released{};
    bool fail_acquire{},fail_submit{};
    std::string acquire_error{"injected acquire failure"};
    std::string submit_error{"injected submit ENOTTY"};
    wsprrypi::Rp1GpclkProviderProgram program{};
    wsprrypi::Rp1GpclkCompletionState state_value{wsprrypi::Rp1GpclkCompletionState::idle};
};

wsprrypi::ExecutionPlan framePlan(std::size_t count=162)
{
    constexpr double spacing=12000.0/8192.0;
    constexpr auto duration=std::chrono::nanoseconds{682666667};
    wsprrypi::ExecutionPlan plan;
    plan.id.value=7; plan.backend=wsprrypi::BackendKind::RP1_GPCLK;
    plan.mode=wsprrypi::TransmissionMode::WSPR;
    plan.reference_frequency_hz=14097100.0;
    for (std::size_t i=0;i<count;++i) {
        wsprrypi::RfEvent event; event.rf_on=true;
        event.offset_from_start=duration*i; event.duration=duration;
        event.frequency_hz=14097100.0-1.5*spacing+(i%4)*spacing;
        plan.events.push_back(event);
    }
    return plan;
}
}

int main()
{
    Owner owner;
    auto provider=std::make_unique<Provider>();
    Provider* observed=provider.get();
    WsprRp1GpclkBackend backend(owner,std::move(provider));
    auto short_plan=framePlan(161);
    expect(!backend.configure(short_plan,{2,4}).ok,"short frame must be rejected");
    auto plan=framePlan();
    expect(!backend.configure(plan,{6,4}).ok,"invalid drive must be rejected");
    expect(backend.configure(plan,{2,4}).ok,"valid frame must configure");
    const auto result=backend.execute(plan);
    expect(result.ok && !result.stopped,"valid frame must execute");
    expect(observed->acquired && observed->submitted && observed->released,"provider lifecycle must complete");
    expect(observed->drive_ma==2,"minimum drive must be carried");
    expect(observed->program.symbols[0]==0 && observed->program.symbols[1]==1,"symbol order must preserve tone indexes");
    expect(observed->program.tones[0].lower_divider_word != observed->program.tones[1].lower_divider_word,"frame must carry distinct tone plans");

    auto acquire_failure_provider=std::make_unique<Provider>();
    Provider* acquire_failure_observed=acquire_failure_provider.get();
    acquire_failure_observed->fail_acquire=true;
    WsprRp1GpclkBackend acquire_failure_backend(owner,std::move(acquire_failure_provider));
    expect(acquire_failure_backend.configure(plan,{2,4}).ok,"acquire-failure frame must configure");
    const auto acquire_failure=acquire_failure_backend.execute(plan);
    expect(!acquire_failure.ok && !acquire_failure.faulted,
        "acquire failure must remain an unsuccessful non-terminal-fault result");
    expect(acquire_failure.error=="injected acquire failure",
        "acquire failure must preserve provider error text");
    expect(!acquire_failure_observed->released,
        "failed acquire must not release ownership that was never acquired");

    auto submit_failure_provider=std::make_unique<Provider>();
    Provider* submit_failure_observed=submit_failure_provider.get();
    submit_failure_observed->fail_submit=true;
    WsprRp1GpclkBackend submit_failure_backend(owner,std::move(submit_failure_provider));
    expect(submit_failure_backend.configure(plan,{2,4}).ok,"submit-failure frame must configure");
    const auto submit_failure=submit_failure_backend.execute(plan);
    expect(!submit_failure.ok && !submit_failure.faulted,
        "submit failure must remain an unsuccessful non-terminal-fault result");
    expect(submit_failure.error=="injected submit ENOTTY",
        "submit failure must preserve provider error text");
    expect(submit_failure_observed->acquired && submit_failure_observed->released,
        "submit failure must release acquired provider ownership");
    if (failures) return 1;
    std::cout << "RP1 GPCLK scheduler backend tests passed\n";
}
