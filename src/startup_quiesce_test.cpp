#include "si5351_device.hpp"
#include "transmission_controller.hpp"
#include "wspr_transmit.hpp"
#include "wspr_transmit_backend_rpi.hpp"
#include "wspr_transmit_backend_si5351.hpp"

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

bool platform_supports_gpio_clock_transmission(std::string*)
{
    return false;
}

void block_signals()
{
}

std::string WsprTransmitter::formatFrequencyMHz(double)
{
    return {};
}

namespace
{
    [[noreturn]] void fail(const std::string& message)
    {
        std::cerr << "startup_quiesce_test: " << message << '\n';
        std::exit(1);
    }

    void expect(bool condition, const std::string& message)
    {
        if (!condition)
            fail(message);
    }

    class TestBridge final : public IControllerBridge
    {
    public:
        WsprTransmitState backendStateValue() const noexcept override
        {
            return WsprTransmitState::ENABLED;
        }
        void backendSetStateValue(WsprTransmitState) noexcept override {}
        bool backendShouldStop() const noexcept override { return false; }
        void backendSignalStopRequest() noexcept override {}
        void backendRequestStopTxNoJoin() noexcept override {}
        bool backendWaitInterruptableFor(std::chrono::nanoseconds) override
        {
            return true;
        }
        void backendThrowIfStopRequested(const char*) override {}
        void backendReportExecutionProgress(std::size_t) noexcept override {}
        void backendFireTransmitCallback(
            WsprTransmissionCallbackEvent,
            WsprTransmitLogLevel,
            const std::string&,
            double) override {}
        bool backendRestartCurrentConfiguration() override { return false; }
    };

    class FakeI2CAdapter final : public Si5351Device::I2CAdapter
    {
    public:
        bool fail_open{false};
        bool fail_select{false};
        bool fail_write{false};
        int open_calls{0};
        int select_calls{0};
        int close_calls{0};
        int read_calls{0};
        std::vector<std::pair<std::uint8_t, std::uint8_t>> writes;

        int openDevice(const std::string&, int) override
        {
            ++open_calls;
            if (fail_open)
            {
                errno = ENOENT;
                return -1;
            }
            return 42;
        }
        int selectSlave(int, std::uint8_t) override
        {
            ++select_calls;
            if (fail_select)
            {
                errno = EIO;
                return -1;
            }
            return 0;
        }
        ssize_t writeData(int, const void* data, std::size_t size) override
        {
            if (fail_write)
            {
                errno = EIO;
                return -1;
            }
            expect(size == 2, "quiesce must use a complete register write");
            const auto* bytes = static_cast<const std::uint8_t*>(data);
            writes.emplace_back(bytes[0], bytes[1]);
            return static_cast<ssize_t>(size);
        }
        ssize_t readData(int, void*, std::size_t) override
        {
            ++read_calls;
            errno = EIO;
            return -1;
        }
        int closeDevice(int) override
        {
            ++close_calls;
            return 0;
        }
    };

    class NoopCompiler final : public wsprrypi::IExecutionPlanCompiler
    {
    public:
        wsprrypi::ExecutionPlan compile(
            const wsprrypi::TransmissionRequest&) const override
        {
            fail("startup quiesce must not compile an execution plan");
        }
    };

    class DispatchBackend final : public wsprrypi::ITransmissionBackend
    {
    public:
        int quiesce_calls{0};
        wsprrypi::BackendInfo info() const override { return {}; }
        wsprrypi::BackendCapabilities capabilities() const override { return {}; }
        wsprrypi::BackendCompileResult configure(
            const wsprrypi::ExecutionPlan&,
            const wsprrypi::BackendExecutionInputs&) override { return {}; }
        wsprrypi::ExecutionResult execute(
            const wsprrypi::ExecutionPlan&) override { return {}; }
        wsprrypi::StartupQuiesceResult quiesceForStartup() override
        {
            ++quiesce_calls;
            return {true, {}};
        }
        void stop() noexcept override {}
        void cleanup() noexcept override {}
    };

    WsprSi5351Backend::Config make_config(
        const std::shared_ptr<FakeI2CAdapter>& adapter,
        bool dry_run)
    {
        WsprSi5351Backend::Config config;
        config.device.i2c_bus = 7;
        config.device.i2c_address = 0x60;
        config.device_adapter = adapter;
        config.dry_run = dry_run;
        return config;
    }

    void test_controller_dispatch()
    {
        NoopCompiler compiler;
        DispatchBackend backend;
        wsprrypi::TransmissionController controller(compiler, backend);
        const auto result = controller.quiesceForStartup();
        expect(result.ok, "controller quiesce dispatch must succeed");
        expect(backend.quiesce_calls == 1,
               "controller must dispatch startup quiesce to the backend");
    }

    void test_si5351_quiesce_success_and_repeat()
    {
        TestBridge bridge;
        auto adapter = std::make_shared<FakeI2CAdapter>();
        WsprSi5351Backend backend(bridge, make_config(adapter, false));

        expect(backend.quiesceForStartup().ok,
               "first Si5351 startup quiesce must succeed");
        expect(backend.quiesceForStartup().ok,
               "repeated Si5351 startup quiesce must succeed");
        expect(adapter->open_calls == 2 && adapter->select_calls == 2,
               "each startup quiesce must open and select the configured I2C device");
        expect(adapter->close_calls == 2,
               "each successful startup quiesce must close the I2C device");
        expect(adapter->read_calls == 0,
               "startup quiesce must not read or program transmission state");
        expect(adapter->writes.size() == 2,
               "startup quiesce must perform exactly one write per call");
        for (const auto& write : adapter->writes)
        {
            expect(write.first == 3 && write.second == 0xff,
                   "startup quiesce must write register 3 = 0xFF only");
        }
    }

    void test_si5351_quiesce_failures_close_handles()
    {
        TestBridge bridge;
        auto write_failure = std::make_shared<FakeI2CAdapter>();
        write_failure->fail_write = true;
        WsprSi5351Backend write_backend(
            bridge, make_config(write_failure, false));
        const auto write_result = write_backend.quiesceForStartup();
        expect(!write_result.ok && !write_result.error.empty(),
               "a register-write failure must report failed quiescence");
        expect(write_failure->close_calls == 1,
               "a register-write failure must close the I2C device");

        auto select_failure = std::make_shared<FakeI2CAdapter>();
        select_failure->fail_select = true;
        WsprSi5351Backend select_backend(
            bridge, make_config(select_failure, false));
        const auto select_result = select_backend.quiesceForStartup();
        expect(!select_result.ok && !select_result.error.empty(),
               "an I2C setup failure must report failed quiescence");
        expect(select_failure->close_calls == 1,
               "an I2C setup failure must close the opened device handle");

        auto open_failure = std::make_shared<FakeI2CAdapter>();
        open_failure->fail_open = true;
        WsprSi5351Backend open_backend(
            bridge, make_config(open_failure, false));
        const auto open_result = open_backend.quiesceForStartup();
        expect(!open_result.ok && !open_result.error.empty(),
               "an I2C open failure must report failed quiescence");
        expect(open_failure->close_calls == 0,
               "an I2C open failure has no device handle to close");
    }

    void test_si5351_dry_run_avoids_i2c()
    {
        TestBridge bridge;
        auto adapter = std::make_shared<FakeI2CAdapter>();
        WsprSi5351Backend backend(bridge, make_config(adapter, true));

        expect(backend.quiesceForStartup().ok,
               "Si5351 dry-run startup quiesce must succeed");
        expect(adapter->open_calls == 0 && adapter->select_calls == 0 &&
                   adapter->close_calls == 0 && adapter->read_calls == 0 &&
                   adapter->writes.empty(),
               "Si5351 dry-run startup quiesce must not access I2C");
    }

    void test_rpi_quiesce_before_first_transmission()
    {
        TestBridge bridge;
        WsprRpiBackend backend(bridge);
        expect(backend.quiesceForStartup().ok,
               "GPIO startup quiesce must be safe before first transmission");
        expect(backend.quiesceForStartup().ok,
               "repeated GPIO startup quiesce must remain safe");
    }
}

int main()
{
    test_controller_dispatch();
    test_si5351_quiesce_success_and_repeat();
    test_si5351_quiesce_failures_close_handles();
    test_si5351_dry_run_avoids_i2c();
    test_rpi_quiesce_before_first_transmission();
    std::cout << "startup_quiesce_test: passed\n";
    return 0;
}
