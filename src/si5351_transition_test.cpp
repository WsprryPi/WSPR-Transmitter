#include "execution_plan.hpp"
#include "wspr_transmit.hpp"
#include "wspr_transmit_backend_si5351.hpp"

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
    constexpr std::uint8_t kOutputEnableRegister = 3;
    constexpr std::uint8_t kPllAParameterBaseRegister = 26;
    constexpr std::uint8_t kPllResetRegister = 177;
    constexpr std::uint8_t kOutputDisableAll = 0xff;
    constexpr std::uint8_t kClk0Enabled = 0xfe;
    int failures = 0;

    void expect(bool condition, const std::string& message)
    {
        if (condition)
            return;
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }

    class FakeI2CAdapter final : public Si5351Device::I2CAdapter
    {
    public:
        int openDevice(const std::string&, int) override
        {
            ++open_calls;
            return 42;
        }

        int selectSlave(int, std::uint8_t) override
        {
            ++select_calls;
            return 0;
        }

        ssize_t writeData(int, const void* data, std::size_t size) override
        {
            const auto* bytes = static_cast<const std::uint8_t*>(data);
            if (size == 1)
            {
                selected_register = bytes[0];
                return 1;
            }
            if (size != 2)
            {
                errno = EIO;
                return -1;
            }

            const std::uint8_t address = bytes[0];
            ++address_attempts[address];
            if (address == fail_address &&
                address_attempts[address] == fail_address_occurrence)
            {
                errno = EIO;
                return -1;
            }

            registers[address] = bytes[1];
            writes.push_back({address, bytes[1]});
            if (after_write)
                after_write(address, bytes[1], address_attempts[address]);
            return 2;
        }

        ssize_t readData(int, void* data, std::size_t size) override
        {
            if (size != 1)
            {
                errno = EIO;
                return -1;
            }
            *static_cast<std::uint8_t*>(data) = registers[selected_register];
            return 1;
        }

        int closeDevice(int) override
        {
            ++close_calls;
            return 0;
        }

        std::array<std::uint8_t, 256> registers{};
        std::array<std::size_t, 256> address_attempts{};
        std::vector<std::pair<std::uint8_t, std::uint8_t>> writes;
        std::uint8_t selected_register = 0;
        std::uint8_t fail_address = 0xff;
        std::size_t fail_address_occurrence = 0;
        std::function<void(std::uint8_t, std::uint8_t, std::size_t)>
            after_write;
        int open_calls = 0;
        int select_calls = 0;
        int close_calls = 0;
    };

    class TestBridge final : public IControllerBridge
    {
    public:
        WsprTransmitState backendStateValue() const noexcept override
        {
            return state_.load();
        }

        void backendSetStateValue(WsprTransmitState state) noexcept override
        {
            state_.store(state);
        }

        bool backendShouldStop() const noexcept override
        {
            return stop_requested_.load();
        }
        void backendSignalStopRequest() noexcept override
        {
            stop_requested_.store(true);
        }
        void backendRequestStopTxNoJoin() noexcept override
        {
            backendSignalStopRequest();
        }
        bool backendWaitInterruptableFor(std::chrono::nanoseconds) override
        {
            ++wait_calls;
            if (interrupt_on_wait_call != 0 &&
                wait_calls == interrupt_on_wait_call)
            {
                backendSignalStopRequest();
                return false;
            }
            return true;
        }
        void backendThrowIfStopRequested(const char*) override {}
        void backendReportExecutionProgress(std::size_t) noexcept override
        {
            ++progress_calls;
        }
        void backendFireTransmitCallback(
            WsprTransmissionCallbackEvent,
            WsprTransmitLogLevel,
            const std::string&,
            double) override
        {
        }
        bool backendRestartCurrentConfiguration() override { return false; }

        std::size_t progress_calls = 0;
        std::size_t wait_calls = 0;
        std::size_t interrupt_on_wait_call = 0;

    private:
        std::atomic<WsprTransmitState> state_{WsprTransmitState::ENABLED};
        std::atomic<bool> stop_requested_{false};
    };

    WsprSi5351Backend::Config config(
        const std::shared_ptr<FakeI2CAdapter>& adapter,
        bool dry_run = false)
    {
        WsprSi5351Backend::Config result;
        result.device.reference_hz = 27000000;
        result.device_adapter = adapter;
        result.planner.reference_hz = 27000000;
        result.planner.tx_output = Si5351Device::Output::CLK0;
        result.power_level = 1;
        result.dry_run = dry_run;
        return result;
    }

    wsprrypi::ExecutionPlan four_tone_plan(
        std::size_t symbol_count,
        std::chrono::nanoseconds symbol_interval =
            std::chrono::nanoseconds::zero(),
        std::chrono::nanoseconds final_duration =
            std::chrono::nanoseconds::zero())
    {
        constexpr double tones_hz[] = {
            144490497.802734375,
            144490499.267578125,
            144490500.732421875,
            144490502.197265625};
        wsprrypi::ExecutionPlan plan;
        plan.id.value = 379;
        plan.request_id.value = 379;
        plan.mode = wsprrypi::TransmissionMode::WSPR;
        plan.backend = wsprrypi::BackendKind::SI5351;
        plan.reference_frequency_hz = 27000000.0;

        for (std::size_t i = 0; i < symbol_count; ++i)
        {
            wsprrypi::RfEvent event;
            event.offset_from_start = symbol_interval *
                static_cast<std::int64_t>(i);
            event.duration = i + 1 == symbol_count
                ? final_duration
                : std::chrono::nanoseconds::zero();
            event.type = wsprrypi::RfEventType::SET_FREQUENCY;
            event.frequency_hz = tones_hz[i % 4];
            event.rf_on = true;
            plan.events.push_back(event);
        }
        plan.summary.event_count = plan.events.size();
        return plan;
    }

    bool configure(
        WsprSi5351Backend& backend,
        const wsprrypi::ExecutionPlan& plan)
    {
        return backend.configure(
            plan,
            wsprrypi::BackendExecutionInputs{1, 0}).ok;
    }

    std::size_t count_write(
        const FakeI2CAdapter& adapter,
        std::uint8_t address,
        std::uint8_t value)
    {
        std::size_t count = 0;
        for (const auto& write : adapter.writes)
        {
            if (write.first == address && write.second == value)
                ++count;
        }
        return count;
    }

    void test_162_symbol_transition_order()
    {
        TestBridge bridge;
        auto adapter = std::make_shared<FakeI2CAdapter>();
        WsprSi5351Backend backend(bridge, config(adapter));
        const wsprrypi::ExecutionPlan plan = four_tone_plan(162);

        expect(configure(backend, plan),
            "four-tone 2 m plan should configure");
        const wsprrypi::ExecutionResult result = backend.execute(plan);
        expect(result.ok && !result.faulted,
            "162-symbol fake-I2C execution should succeed");
        expect(bridge.progress_calls == 162,
            "162-symbol execution should report every symbol");
        expect(adapter->open_calls == 1 && adapter->select_calls == 1 &&
                adapter->close_calls == 1,
            "fake-I2C execution should use one bounded device session");
        expect(count_write(*adapter, kOutputEnableRegister, kClk0Enabled) ==
                162,
            "every 2 m symbol should end with CLK0 enabled");
        expect(adapter->address_attempts[kPllResetRegister] == 163,
            "startup plus every 2 m symbol should reset PLLA");

        bool output_inhibited = true;
        for (const auto& write : adapter->writes)
        {
            if (write.first == kOutputEnableRegister)
                output_inhibited = write.second == kOutputDisableAll;
            if (write.first == kPllAParameterBaseRegister)
            {
                expect(output_inhibited,
                    "every PLL retune must begin while all outputs are "
                    "inhibited");
            }
            if (write.first == kPllResetRegister)
            {
                expect(output_inhibited,
                    "every PLL reset must occur while all outputs are "
                    "inhibited");
            }
        }
        expect(adapter->registers[kOutputEnableRegister] == kOutputDisableAll,
            "successful execution cleanup should leave all outputs disabled");
    }

    void test_transition_failure_stays_inhibited()
    {
        TestBridge bridge;
        auto adapter = std::make_shared<FakeI2CAdapter>();
        adapter->fail_address = 28;
        adapter->fail_address_occurrence = 3;
        WsprSi5351Backend backend(bridge, config(adapter));
        const wsprrypi::ExecutionPlan plan = four_tone_plan(4);

        expect(configure(backend, plan),
            "failure-path 2 m plan should configure before I2C execution");
        const wsprrypi::ExecutionResult result = backend.execute(plan);
        expect(!result.ok && !result.error.empty(),
            "PLL write failure should fail execution with an error");
        expect(bridge.progress_calls == 2,
            "injected second-symbol failure should stop immediately");
        expect(adapter->registers[kOutputEnableRegister] == kOutputDisableAll,
            "PLL write failure and cleanup must leave all outputs disabled");
        expect(count_write(*adapter, kOutputEnableRegister, kClk0Enabled) == 1,
            "failed second transition must not re-enable CLK0");
        expect(adapter->close_calls == 1,
            "PLL write failure should close the device session");
    }

    void expect_interrupted_and_inhibited(
        const wsprrypi::ExecutionResult& result,
        const TestBridge& bridge,
        const FakeI2CAdapter& adapter,
        std::size_t expected_progress,
        const std::string& label)
    {
        expect(result.ok && result.stopped && !result.faulted &&
                result.error.empty(),
            label + " should report a clean interruption");
        expect(bridge.progress_calls == expected_progress,
            label + " should stop at the expected event");
        expect(adapter.registers[kOutputEnableRegister] ==
                kOutputDisableAll,
            label + " should leave all outputs disabled");
        expect(adapter.close_calls == 1,
            label + " should close the device session");
    }

    void test_transition_interruptions_stay_inhibited()
    {
        struct InterruptPoint
        {
            std::uint8_t address;
            std::size_t occurrence;
            const char* label;
        };
        const InterruptPoint points[] = {
            {kOutputEnableRegister, 3, "output-inhibit interruption"},
            {28, 2, "PLL-parameter interruption"},
            {kPllResetRegister, 2, "PLL-reset interruption"}};

        for (const InterruptPoint& point : points)
        {
            TestBridge bridge;
            auto adapter = std::make_shared<FakeI2CAdapter>();
            adapter->after_write =
                [&bridge, point](
                    std::uint8_t address,
                    std::uint8_t,
                    std::size_t occurrence)
                {
                    if (address == point.address &&
                        occurrence == point.occurrence)
                    {
                        bridge.backendSignalStopRequest();
                    }
                };
            WsprSi5351Backend backend(bridge, config(adapter));
            const wsprrypi::ExecutionPlan plan = four_tone_plan(4);

            expect(configure(backend, plan),
                std::string(point.label) + " plan should configure");
            const wsprrypi::ExecutionResult result = backend.execute(plan);
            expect_interrupted_and_inhibited(
                result, bridge, *adapter, 1, point.label);
            expect(count_write(
                    *adapter,
                    kOutputEnableRegister,
                    kClk0Enabled) == 0,
                std::string(point.label) + " must not re-enable CLK0");
        }
    }

    void test_symbol_wait_interruption_cleans_up()
    {
        TestBridge bridge;
        bridge.interrupt_on_wait_call = 1;
        auto adapter = std::make_shared<FakeI2CAdapter>();
        WsprSi5351Backend backend(bridge, config(adapter));
        const wsprrypi::ExecutionPlan plan = four_tone_plan(
            4,
            std::chrono::seconds(1));

        expect(configure(backend, plan),
            "symbol-wait interruption plan should configure");
        const wsprrypi::ExecutionResult result = backend.execute(plan);
        expect_interrupted_and_inhibited(
            result, bridge, *adapter, 1, "symbol-wait interruption");
    }

    void test_final_wait_interruption_cleans_up()
    {
        TestBridge bridge;
        bridge.interrupt_on_wait_call = 1;
        auto adapter = std::make_shared<FakeI2CAdapter>();
        WsprSi5351Backend backend(bridge, config(adapter));
        const wsprrypi::ExecutionPlan plan = four_tone_plan(
            4,
            std::chrono::nanoseconds::zero(),
            std::chrono::seconds(1));

        expect(configure(backend, plan),
            "final-wait interruption plan should configure");
        const wsprrypi::ExecutionResult result = backend.execute(plan);
        expect_interrupted_and_inhibited(
            result, bridge, *adapter, 4, "final-wait interruption");
    }

    void test_162_symbol_dry_run_avoids_i2c()
    {
        TestBridge bridge;
        auto adapter = std::make_shared<FakeI2CAdapter>();
        WsprSi5351Backend backend(bridge, config(adapter, true));
        const wsprrypi::ExecutionPlan plan = four_tone_plan(162);

        expect(configure(backend, plan),
            "dry-run four-tone 2 m plan should configure");
        const wsprrypi::ExecutionResult result = backend.execute(plan);
        expect(result.ok && bridge.progress_calls == 162,
            "dry-run should traverse all 162 symbols");
        expect(adapter->open_calls == 0 && adapter->select_calls == 0 &&
                adapter->close_calls == 0 && adapter->writes.empty(),
            "dry-run 2 m execution must not access I2C");
    }
}

int main()
{
    test_162_symbol_transition_order();
    test_transition_failure_stays_inhibited();
    test_transition_interruptions_stay_inhibited();
    test_symbol_wait_interruption_cleans_up();
    test_final_wait_interruption_cleans_up();
    test_162_symbol_dry_run_avoids_i2c();

    if (failures != 0)
    {
        std::cerr << failures << " Si5351 transition test(s) failed.\n";
        return 1;
    }
    std::cout << "Si5351 transition tests passed.\n";
    return 0;
}
