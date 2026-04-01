#include "wspr_transmit_backend_rpi.hpp"

#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

#include <sys/mman.h>
#include <unistd.h>

#include "bcm_model.hpp"
#include "mailbox.hpp"

namespace
{
    static constexpr size_t NUM_PAGES = 4096;

    static inline int cpu_count() noexcept
    {
        long n = ::sysconf(_SC_NPROCESSORS_ONLN);
        if (n < 1)
            return 1;
        if (n > INT32_MAX)
            return INT32_MAX;
        return static_cast<int>(n);
    }

    static inline int clamp_cpu(int cpu, int ncpu) noexcept
    {
        if (ncpu <= 1)
            return 0;
        if (cpu < 0)
            return 0;
        if (cpu >= ncpu)
            return ncpu - 1;
        return cpu;
    }

    class MailboxMemoryPool
    {
        size_t total_size_;
        uint32_t mem_ref_;
        std::uintptr_t bus_addr_;
        volatile uint8_t *virt_addr_;

    public:
        explicit MailboxMemoryPool(unsigned numpages)
            : total_size_(numpages * Mailbox::PAGE_SIZE),
              mem_ref_(0), bus_addr_(0), virt_addr_(nullptr)
        {
            try
            {
                mem_ref_ = mailbox.memAlloc(total_size_, Mailbox::BLOCK_SIZE);

                bus_addr_ = mailbox.memLock(mem_ref_);
                if (bus_addr_ == 0)
                    throw std::runtime_error("MailboxMemoryPool: memLock failed");

                auto phys = static_cast<off_t>(Mailbox::busToPhysical(bus_addr_));
                virt_addr_ = mailbox.mapMem(phys, total_size_);
                if (virt_addr_ == nullptr)
                    throw std::runtime_error("MailboxMemoryPool: mapMem failed");
            }
            catch (const std::runtime_error &e)
            {
                if (std::string(e.what()).find("timed out") != std::string::npos)
                {
                    throw;
                }

                if (virt_addr_)
                {
                    mailbox.unMapMem(virt_addr_, total_size_);
                    virt_addr_ = nullptr;
                }
                if (bus_addr_)
                {
                    mailbox.memUnlock(mem_ref_);
                    bus_addr_ = 0;
                }
                if (mem_ref_)
                {
                    mailbox.memFree(mem_ref_);
                    mem_ref_ = 0;
                }
                throw;
            }
        }

        ~MailboxMemoryPool()
        {
            if (virt_addr_)
            {
                mailbox.unMapMem(virt_addr_, total_size_);
            }
            if (bus_addr_)
            {
                mailbox.memUnlock(mem_ref_);
                mailbox.memFree(mem_ref_);
            }
        }
    };

    static inline bool gpclk0_wait_not_busy(volatile int &gp0ctl_reg, int max_us)
    {
        const int polls = (max_us <= 0) ? 0 : (max_us / 100);
        for (int i = 0; i < polls; ++i)
        {
            if ((gp0ctl_reg & (1 << 7)) == 0)
                return true;

            struct timespec ts{};
            ts.tv_sec = 0;
            ts.tv_nsec = 100000;
            nanosleep(&ts, nullptr);
        }
        return ((gp0ctl_reg & (1 << 7)) == 0);
    }

    static inline void gpclk0_disable_wait(volatile int &gp0ctl_reg)
    {
        uint32_t ctl = static_cast<uint32_t>(gp0ctl_reg);
        ctl = (ctl & 0x7EFu) | 0x5A000000u;
        gp0ctl_reg = static_cast<int>(ctl);

        if (gpclk0_wait_not_busy(gp0ctl_reg, 200000))
            return;

        ctl = static_cast<uint32_t>(gp0ctl_reg);
        ctl |= (1u << 5);
        ctl |= 0x5A000000u;
        gp0ctl_reg = static_cast<int>(ctl);

        (void)gpclk0_wait_not_busy(gp0ctl_reg, 200000);
    }
}

WsprRpiBackend::DMAConfig::DMAConfig()
    : plld_nominal_freq(500000000.0 * (1 - 2.500e-6)),
      plld_clock_frequency(plld_nominal_freq),
      peripheral_base_virtual(nullptr),
      orig_gp0ctl(0),
      orig_gp0div(0),
      orig_pwm_ctl(0),
      orig_pwm_sta(0),
      orig_pwm_rng1(0),
      orig_pwm_rng2(0),
      orig_pwm_fifocfg(0)
{
}

WsprRpiBackend::WsprRpiBackend(IControllerBridge &owner)
    : owner_(owner)
{
    const int ncpu = cpu_count();
    watchdog_cpu_ = clamp_cpu(watchdog_cpu_, ncpu);

    recovery_stop_.store(false, std::memory_order_release);
    recovery_pending_.store(false, std::memory_order_release);
    recovery_thread_ = std::thread(&WsprRpiBackend::recovery_worker, this);
}

WsprRpiBackend::~WsprRpiBackend()
{
    recovery_stop_.store(true, std::memory_order_release);
    recovery_cv_.notify_all();
    if (recovery_thread_.joinable() &&
        recovery_thread_.get_id() != std::this_thread::get_id())
    {
        recovery_thread_.join();
    }

    stop_watchdog();
}

bool WsprRpiBackend::watchdogFaulted() const noexcept
{
    return watchdog_faulted_.load(std::memory_order_acquire);
}

void WsprRpiBackend::clearWatchdogFault() noexcept
{
    watchdog_faulted_.store(false, std::memory_order_release);
}

void WsprRpiBackend::setWatchdogAutoRecover(bool enable) noexcept
{
    watchdog_auto_recover_.store(enable, std::memory_order_release);
}

bool WsprRpiBackend::watchdogAutoRecoverEnabled() const noexcept
{
    return watchdog_auto_recover_.load(std::memory_order_acquire);
}

bool WsprRpiBackend::recoverFromWatchdogFault()
{
    std::lock_guard<std::mutex> lk(recovery_mtx_);
    return recover_from_watchdog_fault_locked();
}

bool WsprRpiBackend::recoveryInProgress() const noexcept
{
    return recovery_in_progress_.load(std::memory_order_acquire);
}

void WsprRpiBackend::request_watchdog_recovery() noexcept
{
    if (recovery_stop_.load(std::memory_order_acquire))
    {
        return;
    }

    recovery_pending_.store(true, std::memory_order_release);
    recovery_cv_.notify_all();
}

void WsprRpiBackend::recovery_worker()
{
    for (;;)
    {
        std::unique_lock<std::mutex> lk(recovery_wait_mtx_);

        const auto wake_pred =
            [&]()
        {
            if (recovery_stop_.load(std::memory_order_acquire))
            {
                return true;
            }

            if (recovery_pending_.load(std::memory_order_acquire))
            {
                return true;
            }

            if (watchdog_auto_recover_.load(std::memory_order_acquire) &&
                watchdog_faulted_.load(std::memory_order_acquire) &&
                (static_cast<WsprTransmitter::State>(owner_.backendStateValue()) ==
                 WsprTransmitter::State::HUNG))
            {
                std::lock_guard<std::mutex> rlk(recovery_rate_mtx_);
                const auto now_tp = std::chrono::steady_clock::now();
                return (recovery_defer_until_ == std::chrono::steady_clock::time_point{}) ||
                       (now_tp >= recovery_defer_until_);
            }

            return false;
        };

        if (!recovery_stop_.load(std::memory_order_acquire) &&
            !recovery_pending_.load(std::memory_order_acquire))
        {
            std::chrono::steady_clock::time_point defer_tp{};
            {
                std::lock_guard<std::mutex> rlk(recovery_rate_mtx_);
                defer_tp = recovery_defer_until_;
            }

            if (defer_tp != std::chrono::steady_clock::time_point{})
            {
                recovery_cv_.wait_until(lk, defer_tp, wake_pred);
            }
            else
            {
                recovery_cv_.wait(lk, wake_pred);
            }
        }
        else
        {
            recovery_cv_.wait(lk, wake_pred);
        }

        if (recovery_stop_.load(std::memory_order_acquire))
        {
            break;
        }

        recovery_pending_.store(false, std::memory_order_release);
        lk.unlock();

        std::lock_guard<std::mutex> rlk(recovery_mtx_);
        (void)recover_from_watchdog_fault_locked();
    }
}

bool WsprRpiBackend::recover_from_watchdog_fault_locked()
{
    if (!watchdog_faulted_.load(std::memory_order_acquire))
    {
        return false;
    }

    const auto now_tp = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> rlk(recovery_rate_mtx_);

        while (!recovery_attempts_.empty() &&
               (now_tp - recovery_attempts_.front()) > kRecoveryWindow)
        {
            recovery_attempts_.pop_front();
        }

        std::chrono::steady_clock::time_point defer_tp{};

        if (!recovery_attempts_.empty() &&
            (now_tp - recovery_attempts_.back()) < kMinRecoveryInterval)
        {
            defer_tp = recovery_attempts_.back() + kMinRecoveryInterval;
        }
        else if (recovery_attempts_.size() >= kMaxRecoveriesInWindow)
        {
            defer_tp = recovery_attempts_.front() + kRecoveryWindow;
        }

        if (defer_tp != std::chrono::steady_clock::time_point{})
        {
            recovery_defer_until_ = defer_tp;
            const auto defer_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    defer_tp - now_tp)
                    .count();

            {
                std::ostringstream oss;
                oss
                    << "Watchdog recovery deferred (rate limited), "
                    << "retry in " << defer_ms << " ms.";
                owner_.backendFireTransmitCallback(
                    static_cast<int>(WsprTransmitter::TransmissionCallbackEvent::LOGGING),
                    static_cast<int>(WsprTransmitter::LogLevel::ERROR),
                    oss.str(),
                    0.0);
            }

            owner_.backendSetStateValue(static_cast<int>(WsprTransmitter::State::HUNG));
            return false;
        }

        recovery_defer_until_ = std::chrono::steady_clock::time_point{};
        recovery_attempts_.push_back(now_tp);
    }

    const WsprTransmitter::State prior_state =
        static_cast<WsprTransmitter::State>(owner_.backendStateValue());
    if (prior_state == WsprTransmitter::State::DISABLED)
    {
        post_recovery_state_ = static_cast<int>(WsprTransmitter::State::DISABLED);
    }
    else if (prior_state == WsprTransmitter::State::COMPLETE ||
             prior_state == WsprTransmitter::State::CANCELLED)
    {
        post_recovery_state_ = static_cast<int>(prior_state);
    }
    else
    {
        post_recovery_state_ = static_cast<int>(WsprTransmitter::State::ENABLED);
    }

    recovery_in_progress_.store(true, std::memory_order_release);
    owner_.backendSetStateValue(static_cast<int>(WsprTransmitter::State::RECOVERING));

    {
        std::ostringstream oss;
        oss << "Attempting watchdog recovery.";
        owner_.backendFireTransmitCallback(
            static_cast<int>(WsprTransmitter::TransmissionCallbackEvent::LOGGING),
            static_cast<int>(WsprTransmitter::LogLevel::DEBUG),
            oss.str(),
            0.0);
    }

    force_dma_reset_sequence();

    try
    {
        owner_.backendRestartCurrentConfiguration();
        clearWatchdogFault();
        owner_.backendSetStateValue(post_recovery_state_);
        recovery_in_progress_.store(false, std::memory_order_release);
    }
    catch (const std::exception &e)
    {
        recovery_in_progress_.store(false, std::memory_order_release);
        owner_.backendSetStateValue(static_cast<int>(WsprTransmitter::State::HUNG));
        {
            std::ostringstream oss;
            oss << "Watchdog recovery failed: "
                << e.what();
            owner_.backendFireTransmitCallback(
                static_cast<int>(WsprTransmitter::TransmissionCallbackEvent::LOGGING),
                static_cast<int>(WsprTransmitter::LogLevel::ERROR),
                oss.str(),
                0.0);
        }
        return false;
    }

    {
        std::ostringstream oss;
        oss << "Watchdog recovery complete.";
        owner_.backendFireTransmitCallback(
            static_cast<int>(WsprTransmitter::TransmissionCallbackEvent::LOGGING),
            static_cast<int>(WsprTransmitter::LogLevel::DEBUG),
            oss.str(),
            0.0);
    }
    return true;
}

void WsprRpiBackend::start_watchdog()
{
    const int ncpu = cpu_count();

    if (ncpu <= 1)
    {
        {
            std::ostringstream oss;
            oss << "Watchdog disabled (single CPU system).";
            owner_.backendFireTransmitCallback(
                static_cast<int>(WsprTransmitter::TransmissionCallbackEvent::LOGGING),
                static_cast<int>(WsprTransmitter::LogLevel::DEBUG),
                oss.str(),
                0.0);
        }
        return;
    }

    if (watchdog_faulted_.load(std::memory_order_acquire))
    {
        return;
    }

    const bool was_stopped = watchdog_stop_.exchange(false, std::memory_order_acq_rel);
    if (!was_stopped)
    {
        return;
    }

    if (watchdog_thread_.joinable() &&
        watchdog_thread_.get_id() != std::this_thread::get_id())
    {
        watchdog_thread_.join();
    }

    constexpr auto kPollPeriod = std::chrono::milliseconds(20);
    constexpr auto kStallTimeout = std::chrono::milliseconds(250);
    constexpr auto kHeartbeatPeriod = std::chrono::seconds(2);

    std::optional<std::chrono::nanoseconds> inject_stall_after;
    if (const char *env = std::getenv("WSPR_TX_INJECT_WD_STALL"))
    {
        const std::string_view v(env);
        if (!v.empty() && v != "0")
        {
            const bool is_ms = (v.size() >= 2 && v.substr(v.size() - 2) == "ms");
            std::string tmp(v);
            if (is_ms)
                tmp.resize(tmp.size() - 2);

            char *endp = nullptr;
            const long n = std::strtol(tmp.c_str(), &endp, 10);
            if (endp != nullptr && *endp == '\0' && n > 0)
            {
                if (is_ms)
                    inject_stall_after = std::chrono::milliseconds(n);
                else
                    inject_stall_after = std::chrono::seconds(n);
            }
            else if (v == "1")
            {
                inject_stall_after = std::chrono::seconds(0);
            }
        }
    }

    const auto now = std::chrono::steady_clock::now();
    constexpr auto kStartupGrace = std::chrono::milliseconds(750);
    watchdog_last_change_ns_.store(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            (now + kStartupGrace).time_since_epoch())
            .count(),
        std::memory_order_release);
    watchdog_last_conblk_.store(0, std::memory_order_release);
    watchdog_last_txfr_len_.store(0, std::memory_order_release);

    {
        std::ostringstream oss;
        oss << "DMA watchdog started.";
        owner_.backendFireTransmitCallback(
            static_cast<int>(WsprTransmitter::TransmissionCallbackEvent::LOGGING),
            static_cast<int>(WsprTransmitter::LogLevel::DEBUG),
            oss.str(),
            0.0);
    }

    watchdog_thread_ = std::thread(
        [this, kPollPeriod, kStallTimeout, kHeartbeatPeriod, inject_stall_after]
        {
            cpu_set_t cpus;
            CPU_ZERO(&cpus);
            CPU_SET(watchdog_cpu_, &cpus);
            pthread_setaffinity_np(pthread_self(), sizeof(cpus), &cpus);

            sched_param sch{};
            sch.sched_priority = 0;
            pthread_setschedparam(pthread_self(), SCHED_OTHER, &sch);

            auto read_conblk = [this]() -> std::uint32_t
            {
                return static_cast<std::uint32_t>(
                    access_bus_address(DMA_BUS_BASE + 0x04));
            };

            auto read_txfr_len = [this]() -> std::uint32_t
            {
                return static_cast<std::uint32_t>(
                    access_bus_address(DMA_BUS_BASE + 0x14));
            };

            auto read_dma_debug = [this]() -> std::uint32_t
            {
                return static_cast<std::uint32_t>(
                    access_bus_address(DMA_BUS_BASE + 0x20));
            };

            auto read_dma_nextconbk = [this]() -> std::uint32_t
            {
                return static_cast<std::uint32_t>(
                    access_bus_address(DMA_BUS_BASE + 0x1C));
            };

            auto read_pwm_ctl = [this]() -> std::uint32_t
            {
                return static_cast<std::uint32_t>(
                    access_bus_address(PWM_BUS_BASE + 0x00));
            };

            auto read_pwm_sta = [this]() -> std::uint32_t
            {
                return static_cast<std::uint32_t>(
                    access_bus_address(PWM_BUS_BASE + 0x04));
            };

            auto read_pwm_dmac = [this]() -> std::uint32_t
            {
                return static_cast<std::uint32_t>(
                    access_bus_address(PWM_BUS_BASE + 0x08));
            };

            auto read_gp0ctl = [this]() -> std::uint32_t
            {
                return static_cast<std::uint32_t>(
                    access_bus_address(CM_GP0CTL_BUS));
            };

            auto read_gp0div = [this]() -> std::uint32_t
            {
                return static_cast<std::uint32_t>(
                    access_bus_address(CM_GP0DIV_BUS));
            };

            auto read_cs = [this]() -> std::uint32_t
            {
                return static_cast<std::uint32_t>(
                    access_bus_address(DMA_BUS_BASE + 0x00));
            };

            auto last_heartbeat = std::chrono::steady_clock::now();
            std::uint32_t last_heartbeat_conblk = 0;
            std::uint32_t last_heartbeat_txfr_len = 0;

            bool injected = false;
            std::optional<std::chrono::steady_clock::time_point> tx_start;
            std::uint32_t injected_conblk = 0;
            std::uint32_t injected_txfr_len = 0;

            while (!watchdog_stop_.load(std::memory_order_acquire))
            {
                if (static_cast<WsprTransmitter::State>(owner_.backendStateValue()) !=
                    WsprTransmitter::State::TRANSMITTING)
                {
                    std::this_thread::sleep_for(kPollPeriod);
                    continue;
                }

                const std::uint32_t cs = read_cs();
                const bool active = (cs & 0x1u) != 0u;

                if (!active)
                {
                    const auto ts = std::chrono::steady_clock::now();
                    watchdog_last_change_ns_.store(
                        std::chrono::duration_cast<std::chrono::nanoseconds>(
                            ts.time_since_epoch())
                            .count(),
                        std::memory_order_release);
                    watchdog_last_conblk_.store(read_conblk(), std::memory_order_release);
                    watchdog_last_txfr_len_.store(read_txfr_len(), std::memory_order_release);
                    std::this_thread::sleep_for(kPollPeriod);
                    continue;
                }

                const auto now_tp = std::chrono::steady_clock::now();
                if (!tx_start.has_value())
                    tx_start = now_tp;

                if (!injected && inject_stall_after.has_value())
                {
                    const auto elapsed = now_tp - *tx_start;
                    const auto after =
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            *inject_stall_after);
                    if (elapsed >= after)
                    {
                        injected = true;
                        injected_conblk = read_conblk();
                        injected_txfr_len = read_txfr_len();

                        std::ostringstream oss;
                        oss << "DMA watchdog: injecting stall."
                            << " after="
                            << std::chrono::duration_cast<std::chrono::milliseconds>(
                                   *inject_stall_after)
                                   .count()
                            << " ms";
                        owner_.backendFireTransmitCallback(
                            static_cast<int>(WsprTransmitter::TransmissionCallbackEvent::LOGGING),
                            static_cast<int>(WsprTransmitter::LogLevel::DEBUG),
                            oss.str(),
                            0.0);
                    }
                }

                const std::uint32_t conblk = injected ? injected_conblk : read_conblk();
                const std::uint32_t txfr_len = injected ? injected_txfr_len : read_txfr_len();

                const std::uint32_t last_conblk =
                    watchdog_last_conblk_.load(std::memory_order_acquire);
                const std::uint32_t last_txfr_len =
                    watchdog_last_txfr_len_.load(std::memory_order_acquire);

                if (conblk != last_conblk || txfr_len != last_txfr_len)
                {
                    const auto ts = std::chrono::steady_clock::now();
                    watchdog_last_conblk_.store(conblk, std::memory_order_release);
                    watchdog_last_txfr_len_.store(txfr_len, std::memory_order_release);
                    watchdog_last_change_ns_.store(
                        std::chrono::duration_cast<std::chrono::nanoseconds>(
                            ts.time_since_epoch())
                            .count(),
                        std::memory_order_release);
                    std::this_thread::sleep_for(kPollPeriod);
                    continue;
                }

                const auto now_ns =
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        now_tp.time_since_epoch())
                        .count();
                const auto last_ns = watchdog_last_change_ns_.load(std::memory_order_acquire);
                const auto stalled_for = std::chrono::nanoseconds(now_ns - last_ns);

                if ((now_tp - last_heartbeat) >= kHeartbeatPeriod)
                {
                    const bool advancing =
                        (conblk != last_heartbeat_conblk) ||
                        (txfr_len != last_heartbeat_txfr_len);
                    std::ostringstream oss;
                    oss
                        << "DMA watchdog: CS=0x" << std::hex << cs
                        << " CONBLK_AD=0x" << conblk
                        << " TXFR_LEN=0x" << txfr_len
                        << std::dec
                        << (advancing ? " advancing" : " not-advancing")
                        << " stalled_for="
                        << std::chrono::duration_cast<std::chrono::milliseconds>(
                               stalled_for)
                               .count()
                        << " ms";
                    owner_.backendFireTransmitCallback(
                        static_cast<int>(WsprTransmitter::TransmissionCallbackEvent::LOGGING),
                        static_cast<int>(WsprTransmitter::LogLevel::DEBUG),
                        oss.str(),
                        0.0);
                    last_heartbeat = now_tp;
                    last_heartbeat_conblk = conblk;
                    last_heartbeat_txfr_len = txfr_len;
                }

                if (stalled_for >= kStallTimeout)
                {
                    watchdog_faulted_.store(true, std::memory_order_release);
                    owner_.backendSetStateValue(static_cast<int>(WsprTransmitter::State::HUNG));

                    {
                        std::ostringstream oss;
                        oss << "DMA watchdog detected a stall."
                            << " CS=0x" << std::hex << cs
                            << " CONBLK_AD=0x" << conblk
                            << std::dec;
                        owner_.backendFireTransmitCallback(
                            static_cast<int>(WsprTransmitter::TransmissionCallbackEvent::LOGGING),
                            static_cast<int>(WsprTransmitter::LogLevel::ERROR),
                            oss.str(),
                            0.0);
                    }

                    try
                    {
                        const std::uint32_t debug = read_dma_debug();
                        const std::uint32_t nextconbk = read_dma_nextconbk();
                        const std::uint32_t pwm_ctl = read_pwm_ctl();
                        const std::uint32_t pwm_sta = read_pwm_sta();
                        const std::uint32_t pwm_dmac = read_pwm_dmac();
                        const std::uint32_t gp0ctl = read_gp0ctl();
                        const std::uint32_t gp0div = read_gp0div();

                        std::ostringstream flags;
                        flags
                            << ((cs & (1u << 0)) ? " ACTIVE" : "")
                            << ((cs & (1u << 1)) ? " END" : "")
                            << ((cs & (1u << 2)) ? " INT" : "")
                            << ((cs & (1u << 3)) ? " DREQ" : "")
                            << ((cs & (1u << 4)) ? " PAUSED" : "")
                            << ((cs & (1u << 5)) ? " DREQ_STOPS" : "")
                            << ((cs & (1u << 6)) ? " WAIT_OUTSTANDING" : "")
                            << ((cs & (1u << 8)) ? " ERROR" : "")
                            << ((cs & (1u << 9)) ? " WAIT_RESP" : "")
                            << ((cs & (1u << 28)) ? " WIDE_BURSTS" : "")
                            << ((cs & (1u << 29)) ? " DISDEBUG" : "")
                            << ((cs & (1u << 30)) ? " ABORT" : "")
                            << ((cs & (1u << 31)) ? " RESET" : "");

                        std::ostringstream oss;
                        oss
                            << "DMA watchdog diagnostics:"
                            << " CS=0x" << std::hex << cs
                            << " (" << flags.str() << " )"
                            << " CONBLK_AD=0x" << conblk
                            << " NEXTCONBK=0x" << nextconbk
                            << " TXFR_LEN=0x" << txfr_len
                            << " DEBUG=0x" << debug
                            << " PWM_CTL=0x" << pwm_ctl
                            << " PWM_STA=0x" << pwm_sta
                            << " PWM_DMAC=0x" << pwm_dmac
                            << " GP0CTL=0x" << gp0ctl
                            << " GP0DIV=0x" << gp0div
                            << std::dec;
                        owner_.backendFireTransmitCallback(
                            static_cast<int>(WsprTransmitter::TransmissionCallbackEvent::LOGGING),
                            static_cast<int>(WsprTransmitter::LogLevel::DEBUG),
                            oss.str(),
                            0.0);
                    }
                    catch (...)
                    {
                    }

                    owner_.backendRequestStopTxNoJoin();
                    force_dma_reset_sequence();
                    request_watchdog_recovery();
                    return;
                }

                std::this_thread::sleep_for(kPollPeriod);
            }
        });
}

void WsprRpiBackend::stop_watchdog()
{
    const bool was_running = !watchdog_stop_.exchange(true, std::memory_order_acq_rel);
    if (!was_running)
    {
        return;
    }

    if (watchdog_thread_.joinable() &&
        watchdog_thread_.get_id() != std::this_thread::get_id())
    {
        watchdog_thread_.join();
    }

    {
        std::ostringstream oss;
        oss << "DMA watchdog stopped.";
        owner_.backendFireTransmitCallback(
            static_cast<int>(WsprTransmitter::TransmissionCallbackEvent::LOGGING),
            static_cast<int>(WsprTransmitter::LogLevel::DEBUG),
            oss.str(),
            0.0);
    }
}

void WsprRpiBackend::force_dma_reset_sequence() noexcept
{
    if (dma_config_.peripheral_base_virtual == nullptr)
    {
        return;
    }

    try
    {
        volatile DMAregs *DMA0 =
            reinterpret_cast<volatile DMAregs *>(&(access_bus_address(DMA_BUS_BASE)));

        DMA0->CS = (1u << 30) | (1u << 31);
        usleep(10);
        DMA0->CS = 1u << 31;
        DMA0->CONBLK_AD = 0;
        DMA0->TI = 0;
        DMA0->SOURCE_AD = 0;
        DMA0->DEST_AD = 0;
        DMA0->TXFR_LEN = 0;
        DMA0->STRIDE = 0;
        DMA0->NEXTCONBK = 0;
        DMA0->DEBUG = 7u;

        access_bus_address(PWM_BUS_BASE + 0x00) = 0;
        access_bus_address(PWM_BUS_BASE + 0x08) = 0;

        disable_clock();
    }
    catch (...)
    {
    }
}

void WsprRpiBackend::dma_cleanup()
{
    if (!dma_setup_done_)
    {
        return;
    }
    dma_setup_done_ = false;

    if (!dma_config_.peripheral_base_virtual)
    {
        return;
    }

    disable_hardware_sequence();

    access_bus_address(CM_GP0DIV_BUS) = dma_config_.orig_gp0div;
    access_bus_address(CM_GP0CTL_BUS) = dma_config_.orig_gp0ctl;
    access_bus_address(PWM_BUS_BASE + 0x00) = dma_config_.orig_pwm_ctl;
    access_bus_address(PWM_BUS_BASE + 0x04) = dma_config_.orig_pwm_sta;
    access_bus_address(PWM_BUS_BASE + 0x10) = dma_config_.orig_pwm_rng1;
    access_bus_address(PWM_BUS_BASE + 0x20) = dma_config_.orig_pwm_rng2;
    access_bus_address(PWM_BUS_BASE + 0x08) = dma_config_.orig_pwm_fifocfg;

    if (dma_config_.peripheral_base_virtual)
    {
        ::mailbox.unMapMem(
            dma_config_.peripheral_base_virtual,
            Mailbox::PAGE_SIZE * NUM_PAGES);
        dma_config_.peripheral_base_virtual = nullptr;
    }

    deallocate_memory_pool();
    mailbox.close();

    dma_config_ = DMAConfig();
    mailbox_struct_ = MailboxStruct();
}

int WsprRpiBackend::get_gpio_power_mw(int level)
{
    if (level < 0 || level >= static_cast<int>(DRIVE_STRENGTH_TABLE.size()))
    {
        throw std::out_of_range(
            "WsprTransmitter::get_gpio_power_mw: Drive strength level "
            "must be between 0 and 7");
    }
    return DRIVE_STRENGTH_TABLE[level];
}

inline volatile int &WsprRpiBackend::access_bus_address(std::uintptr_t bus_addr)
{
    std::uintptr_t offset = Mailbox::offsetFromBase(bus_addr);
    return *reinterpret_cast<volatile int *>(dma_config_.peripheral_base_virtual + offset);
}

inline void WsprRpiBackend::set_bit_bus_address(std::uintptr_t base, unsigned int bit)
{
    access_bus_address(base) |= 1 << bit;
}

inline void WsprRpiBackend::clear_bit_bus_address(std::uintptr_t base, unsigned int bit)
{
    access_bus_address(base) &= ~(1 << bit);
}

void WsprRpiBackend::get_plld()
{
    static std::optional<unsigned> cached_revision;
    if (!cached_revision)
    {
        std::ifstream file("/proc/cpuinfo");
        if (file)
        {
            std::string line;
            unsigned value = 0;
            const std::string pattern = "Revision\t: %x";
            while (std::getline(file, line))
            {
                if (sscanf(line.c_str(), pattern.c_str(), &value) == 1)
                {
                    cached_revision = value;
                    break;
                }
            }
        }
        if (!cached_revision)
        {
            cached_revision = 0;
        }
    }

    unsigned rev = *cached_revision;
    BCMChip proc_id;

    if (rev & 0x800000)
    {
        auto raw = (rev & 0xF000) >> 12;
        proc_id = static_cast<BCMChip>(raw);
    }
    else
    {
        proc_id = BCMChip::BCM_HOST_PROCESSOR_BCM2835;
    }

    double base_freq_hz = 500e6;
    switch (proc_id)
    {
    case BCMChip::BCM_HOST_PROCESSOR_BCM2835:
    case BCMChip::BCM_HOST_PROCESSOR_BCM2836:
    case BCMChip::BCM_HOST_PROCESSOR_BCM2837:
        base_freq_hz = 500e6;
        break;

    case BCMChip::BCM_HOST_PROCESSOR_BCM2711:
        base_freq_hz = 750e6;
        break;

    default:
        throw std::runtime_error(
            std::string("Error: Unknown chipset (") +
            std::string(to_string(proc_id)) + ")");
    }

    dma_config_.plld_nominal_freq = base_freq_hz;
    dma_config_.plld_clock_frequency = base_freq_hz;

    if (dma_config_.plld_clock_frequency <= 0)
    {
        std::ostringstream oss;
        oss << "Error: Invalid PLLD frequency; defaulting to 500 MHz";
        owner_.backendFireTransmitCallback(
            static_cast<int>(WsprTransmitter::TransmissionCallbackEvent::LOGGING),
            static_cast<int>(WsprTransmitter::LogLevel::ERROR),
            oss.str(),
            0.0);
        dma_config_.plld_nominal_freq = 500e6;
        dma_config_.plld_clock_frequency = 500e6;
    }
}

void WsprRpiBackend::allocate_memory_pool(unsigned numpages)
{
    mailbox_struct_.mem_ref = mailbox.memAlloc(
        Mailbox::PAGE_SIZE * numpages,
        Mailbox::BLOCK_SIZE);
    if (mailbox_struct_.mem_ref == 0)
    {
        throw std::runtime_error("Error: memAlloc failed.");
    }

    mailbox_struct_.bus_addr = mailbox.memLock(mailbox_struct_.mem_ref);
    if (mailbox_struct_.bus_addr == 0)
    {
        mailbox.memFree(mailbox_struct_.mem_ref);
        throw std::runtime_error("Error: memLock failed.");
    }

    mailbox_struct_.virt_addr = mailbox.mapMem(
        mailbox.busToPhysical(mailbox_struct_.bus_addr),
        Mailbox::PAGE_SIZE * numpages);
    if (mailbox_struct_.virt_addr == nullptr)
    {
        mailbox.memUnlock(mailbox_struct_.mem_ref);
        mailbox.memFree(mailbox_struct_.mem_ref);
        throw std::runtime_error("Error: mapMem failed.");
    }

    mailbox_struct_.pool_size = numpages;
    mailbox_struct_.pool_cnt = 0;
}

void WsprRpiBackend::get_real_mem_page_from_pool(void **vAddr, void **bAddr)
{
    if (mailbox_struct_.pool_cnt >= mailbox_struct_.pool_size)
    {
        throw std::runtime_error("Error: unable to allocate more pages.");
    }

    unsigned offset = mailbox_struct_.pool_cnt * Mailbox::PAGE_SIZE;

    *vAddr = reinterpret_cast<void *>(
        reinterpret_cast<uintptr_t>(mailbox_struct_.virt_addr) + offset);
    *bAddr = reinterpret_cast<void *>(
        reinterpret_cast<uintptr_t>(mailbox_struct_.bus_addr) + offset);

    mailbox_struct_.pool_cnt++;
}

void WsprRpiBackend::deallocate_memory_pool()
{
    if (mailbox_struct_.virt_addr != nullptr)
    {
        mailbox.unMapMem(
            mailbox_struct_.virt_addr,
            mailbox_struct_.pool_size * Mailbox::PAGE_SIZE);
        mailbox_struct_.virt_addr = nullptr;
    }

    if (mailbox_struct_.mem_ref != 0)
    {
        mailbox.memUnlock(mailbox_struct_.mem_ref);
        mailbox.memFree(mailbox_struct_.mem_ref);
        mailbox_struct_.mem_ref = 0;
    }

    mailbox_struct_.pool_size = 0;
    mailbox_struct_.pool_cnt = 0;
}

void WsprRpiBackend::disable_hardware_sequence()
{
    const bool was_on =
        (static_cast<WsprTransmitter::State>(owner_.backendStateValue()) ==
         WsprTransmitter::State::TRANSMITTING);

    if (was_on && dma_config_.peripheral_base_virtual != nullptr)
    {
        const std::uint32_t conblk =
            static_cast<std::uint32_t>(access_bus_address(DMA_BUS_BASE + 0x04));
        const std::uint32_t cs =
            static_cast<std::uint32_t>(access_bus_address(DMA_BUS_BASE + 0x00));

        std::ostringstream oss;
        oss << "DMA before off: CS=0x"
            << std::hex << cs
            << " CONBLK_AD=0x" << conblk
            << std::dec;
        owner_.backendFireTransmitCallback(
            static_cast<int>(WsprTransmitter::TransmissionCallbackEvent::LOGGING),
            static_cast<int>(WsprTransmitter::LogLevel::DEBUG),
            oss.str(),
            0.0);
    }

    if (dma_config_.peripheral_base_virtual == nullptr)
    {
        if (!recovery_in_progress_.load(std::memory_order_acquire))
        {
            owner_.backendSetStateValue(static_cast<int>(WsprTransmitter::State::ENABLED));
        }
        return;
    }

    volatile DMAregs *DMA0 =
        reinterpret_cast<volatile DMAregs *>(&(access_bus_address(DMA_BUS_BASE)));
    DMA0->CS = 1u << 31;

    access_bus_address(PWM_BUS_BASE + 0x00) = 0;
    access_bus_address(PWM_BUS_BASE + 0x08) = 0;

    disable_clock();
}

void WsprRpiBackend::disable_clock()
{
    if (!recovery_in_progress_.load(std::memory_order_acquire))
    {
        owner_.backendSetStateValue(static_cast<int>(WsprTransmitter::State::ENABLED));
    }

    if (dma_config_.peripheral_base_virtual == nullptr)
        return;

    gpclk0_disable_wait(access_bus_address(CM_GP0CTL_BUS));
}

void WsprRpiBackend::transmit_on()
{
    set_bit_bus_address(GPIO_BUS_BASE, 14);
    clear_bit_bus_address(GPIO_BUS_BASE, 13);
    clear_bit_bus_address(GPIO_BUS_BASE, 12);

    access_bus_address(PADS_GPIO_0_27_BUS) = 0x5a000018 + owner_.backendPowerLevel();

    struct GPCTL setupword = {6, 0, 0, 0, 0, 3, 0x5A};
    setupword = {6, 1, 0, 0, 0, 3, 0x5A};
    int temp;
    std::memcpy(&temp, &setupword, sizeof(int));

    access_bus_address(CM_GP0CTL_BUS) = temp;
    owner_.backendSetStateValue(static_cast<int>(WsprTransmitter::State::TRANSMITTING));
}

void WsprRpiBackend::transmit_off()
{
    stop_watchdog();
    disable_hardware_sequence();
}

void WsprRpiBackend::transmit_symbol(
    const std::uint32_t &sym_num,
    const double &tsym,
    std::uint32_t &bufPtr,
    int symbol_index)
{
    if (owner_.backendShouldStop())
    {
        return;
    }

    constexpr std::uint32_t kMask = 0x3FFu;
    constexpr std::uint32_t kLead = 64u;
    constexpr int kPollSleepUs = 50;
    constexpr int kMaxWaitUs = 200000;

    auto dma_conblk_ad = [&]() -> std::uint32_t
    {
        return static_cast<std::uint32_t>(
            access_bus_address(DMA_BUS_BASE + 0x04));
    };

    auto wait_cb_not_active = [&](std::uint32_t idx) -> bool
    {
        const std::uint32_t target =
            static_cast<std::uint32_t>(instructions_[idx].b);

        int waited_us = 0;
        while (!owner_.backendShouldStop())
        {
            const std::uint32_t cur = dma_conblk_ad();
            if (cur != target)
                return true;

            if (waited_us >= kMaxWaitUs)
            {
                std::ostringstream oss;
                oss << "DMA appears stuck at CONBLK_AD=0x"
                    << std::hex << cur << std::dec
                    << ", forcing stop to avoid deadlock.";
                owner_.backendFireTransmitCallback(
                    static_cast<int>(WsprTransmitter::TransmissionCallbackEvent::LOGGING),
                    static_cast<int>(WsprTransmitter::LogLevel::DEBUG),
                    oss.str(),
                    0.0);

                owner_.backendSignalStopRequest();
                return false;
            }

            usleep(kPollSleepUs);
            waited_us += kPollSleepUs;
        }
        return false;
    };

    auto advance_with_lead = [&]() -> void
    {
        bufPtr = (bufPtr + kLead) & kMask;
    };

    const bool is_tone = (tsym == 0.0);
    const int f0_idx = static_cast<int>(sym_num) * 2;
    const int f1_idx = f0_idx + 1;

    const std::int64_t pwm_clocks_per_iter =
        static_cast<std::int64_t>(PWM_CLOCKS_PER_ITER_NOMINAL);

    advance_with_lead();

    if (is_tone)
    {
        while (!owner_.backendShouldStop())
        {
            const std::uint32_t n_pwmclk =
                static_cast<std::uint32_t>(pwm_clocks_per_iter);

            bufPtr = (bufPtr + 1) & kMask;
            if (!wait_cb_not_active(bufPtr))
                return;

            reinterpret_cast<CB *>(instructions_[bufPtr].v)->SOURCE_AD =
                static_cast<std::uint32_t>(
                    static_cast<std::uintptr_t>(const_page_.b) +
                    static_cast<std::uintptr_t>(f0_idx * 4));

            bufPtr = (bufPtr + 1) & kMask;
            if (!wait_cb_not_active(bufPtr))
                return;

            reinterpret_cast<CB *>(instructions_[bufPtr].v)->TXFR_LEN = n_pwmclk;
        }

        return;
    }

    const std::uint32_t table_size = 1024;
    const double pwm_sample_hz =
        pwm_clock_init_ /
        static_cast<double>(table_size);
    const std::int64_t n_pwmclk_per_sym =
        static_cast<std::int64_t>(std::llround(pwm_sample_hz * tsym));

    {
        const int total_symbols =
            static_cast<int>(owner_.backendSymbolCount());

        std::ostringstream oss;
        oss
            << "sym=" << sym_num
            << " idx=";

        if (symbol_index >= 0)
        {
            oss
                << std::setw(3) << std::setfill('0') << (symbol_index + 1)
                << "/" << total_symbols;
        }
        else
        {
            oss << "-";
        }

        oss
            << " tsym=" << std::fixed << std::setprecision(6) << tsym
            << " pwm_clock_init_=" << std::fixed << std::setprecision(3)
            << pwm_clock_init_ << std::defaultfloat
            << " n_pwmclk_per_sym=" << n_pwmclk_per_sym
            << " pwm_clocks_per_iter=" << pwm_clocks_per_iter;

        owner_.backendFireTransmitCallback(
                    static_cast<int>(WsprTransmitter::TransmissionCallbackEvent::LOGGING),
                    static_cast<int>(WsprTransmitter::LogLevel::DEBUG),
                    oss.str(),
                    0.0);
    }

    if (n_pwmclk_per_sym <= 0 || n_pwmclk_per_sym > 5'000'000'000LL)
        throw std::runtime_error(
            "transmit_symbol(): invalid n_pwmclk_per_sym (bad PWM clock).");

    std::int64_t n_pwmclk_transmitted = 0;
    std::int64_t n_f0_transmitted = 0;

    const double f0_freq =
        dma_config_.plld_clock_frequency /
        (static_cast<double>(
             reinterpret_cast<std::uint32_t *>(const_page_.v)[f0_idx] & 0x00FFFFFFu) /
         std::pow(2.0, 12));
    const double f1_freq =
        dma_config_.plld_clock_frequency /
        (static_cast<double>(
             reinterpret_cast<std::uint32_t *>(const_page_.v)[f1_idx] & 0x00FFFFFFu) /
         std::pow(2.0, 12));
    const double tone_freq =
        owner_.backendFrequency() - 1.5 * owner_.backendToneSpacing() +
        static_cast<double>(sym_num) * owner_.backendToneSpacing();

    const double f0_ratio =
        1.0 - (tone_freq - f0_freq) / (f1_freq - f0_freq);

    while (n_pwmclk_transmitted < n_pwmclk_per_sym &&
           !owner_.backendShouldStop())
    {
        std::int64_t n_pwmclk = pwm_clocks_per_iter;

        n_pwmclk += static_cast<std::int64_t>(std::llround(
            (std::rand() / (RAND_MAX + 1.0) - 0.5) *
            static_cast<double>(n_pwmclk)));

        if (n_pwmclk <= 0)
            n_pwmclk = 1;

        if (n_pwmclk_transmitted + n_pwmclk > n_pwmclk_per_sym)
            n_pwmclk = n_pwmclk_per_sym - n_pwmclk_transmitted;

        std::int64_t n_f0 =
            static_cast<std::int64_t>(std::llround(
                f0_ratio * static_cast<double>(n_pwmclk_transmitted + n_pwmclk))) -
            n_f0_transmitted;

        if (n_f0 < 0)
            n_f0 = 0;
        if (n_f0 > n_pwmclk)
            n_f0 = n_pwmclk;

        const std::int64_t n_f1 = n_pwmclk - n_f0;

        bufPtr = (bufPtr + 1) & kMask;
        if (!wait_cb_not_active(bufPtr))
            return;

        reinterpret_cast<CB *>(instructions_[bufPtr].v)->SOURCE_AD =
            static_cast<std::uint32_t>(
                static_cast<std::uintptr_t>(const_page_.b) +
                static_cast<std::uintptr_t>(f0_idx * 4));

        bufPtr = (bufPtr + 1) & kMask;
        if (!wait_cb_not_active(bufPtr))
            return;

        reinterpret_cast<CB *>(instructions_[bufPtr].v)->TXFR_LEN =
            static_cast<std::uint32_t>(n_f0);

        bufPtr = (bufPtr + 1) & kMask;
        if (!wait_cb_not_active(bufPtr))
            return;

        reinterpret_cast<CB *>(instructions_[bufPtr].v)->SOURCE_AD =
            static_cast<std::uint32_t>(
                static_cast<std::uintptr_t>(const_page_.b) +
                static_cast<std::uintptr_t>(f1_idx * 4));

        bufPtr = (bufPtr + 1) & kMask;
        if (!wait_cb_not_active(bufPtr))
            return;

        reinterpret_cast<CB *>(instructions_[bufPtr].v)->TXFR_LEN =
            static_cast<std::uint32_t>(n_f1);

        n_pwmclk_transmitted += n_pwmclk;
        n_f0_transmitted += n_f0;
    }
}

double WsprRpiBackend::bit_trunc(const double &d, const int &lsb)
{
    const double factor = std::pow(2.0, lsb);
    return std::floor(d / factor) * factor;
}

void WsprRpiBackend::create_dma_pages(
    PageInfo &const_page,
    PageInfo &instr_page,
    PageInfo instructions[])
{
    allocate_memory_pool(1025);

    {
        void *tmp_v, *tmp_b;
        get_real_mem_page_from_pool(&tmp_v, &tmp_b);
        const_page.v = tmp_v;
        const_page.b = reinterpret_cast<std::uintptr_t>(tmp_b);
    }

    int instrCnt = 0;

    while (instrCnt < 1024)
    {
        {
            void *tmp_v, *tmp_b;
            get_real_mem_page_from_pool(&tmp_v, &tmp_b);
            instr_page.v = tmp_v;
            instr_page.b = reinterpret_cast<std::uintptr_t>(tmp_b);
        }

        struct CB *instr0 = reinterpret_cast<struct CB *>(instr_page.v);

        for (int i = 0; i < static_cast<int>(Mailbox::PAGE_SIZE / sizeof(struct CB)); i++)
        {
            instructions[instrCnt].v = static_cast<void *>(
                static_cast<char *>(instr_page.v) + sizeof(struct CB) * i);
            instructions[instrCnt].b = instr_page.b + static_cast<std::uintptr_t>(
                                                          sizeof(struct CB) * i);

            instr0->SOURCE_AD = static_cast<uint32_t>(const_page.b + 2048);
            instr0->DEST_AD = PWM_BUS_BASE + 0x18;
            instr0->TXFR_LEN = 4;
            instr0->STRIDE = 0;
            instr0->TI = (1 << 6) | (5 << 16) | (1 << 26);
            instr0->RES1 = 0;
            instr0->RES2 = 0;

            if (i % 2)
            {
                instr0->DEST_AD = CM_GP0DIV_BUS;
                instr0->STRIDE = 4;
                instr0->TI = (1 << 26);
            }

            if (instrCnt != 0)
            {
                reinterpret_cast<volatile CB *>(instructions[instrCnt - 1].v)
                    ->NEXTCONBK = static_cast<uint32_t>(instructions[instrCnt].b);
            }

            instr0++;
            instrCnt++;
        }
    }

    reinterpret_cast<volatile CB *>(instructions[1023].v)
        ->NEXTCONBK = static_cast<uint32_t>(instructions[0].b);
    reinterpret_cast<volatile CB *>(instructions[1023].v)
        ->NEXTCONBK = static_cast<uint32_t>(instructions[0].b);

    access_bus_address(CLK_BUS_BASE + 40 * 4) = 0x5A000026;
    owner_.backendThrowIfStopRequested("waiting for hardware");
    (void)owner_.backendWaitInterruptableFor(std::chrono::milliseconds(1));
    owner_.backendThrowIfStopRequested("waiting for hardware");
    access_bus_address(CLK_BUS_BASE + 41 * 4) = 0x5A002000;
    access_bus_address(CLK_BUS_BASE + 40 * 4) = 0x5A000016;
    owner_.backendThrowIfStopRequested("waiting for hardware");
    (void)owner_.backendWaitInterruptableFor(std::chrono::milliseconds(1));
    owner_.backendThrowIfStopRequested("waiting for hardware");

    access_bus_address(PWM_BUS_BASE + 0x0) = 0;
    owner_.backendThrowIfStopRequested("waiting for hardware");
    (void)owner_.backendWaitInterruptableFor(std::chrono::milliseconds(1));
    owner_.backendThrowIfStopRequested("waiting for hardware");
    access_bus_address(PWM_BUS_BASE + 0x4) = -1;
    owner_.backendThrowIfStopRequested("waiting for hardware");
    (void)owner_.backendWaitInterruptableFor(std::chrono::milliseconds(1));
    owner_.backendThrowIfStopRequested("waiting for hardware");
    access_bus_address(PWM_BUS_BASE + 0x10) = 32;
    access_bus_address(PWM_BUS_BASE + 0x20) = 32;
    access_bus_address(PWM_BUS_BASE + 0x0) = -1;
    owner_.backendThrowIfStopRequested("waiting for hardware");
    (void)owner_.backendWaitInterruptableFor(std::chrono::milliseconds(1));
    owner_.backendThrowIfStopRequested("waiting for hardware");
    access_bus_address(PWM_BUS_BASE + 0x8) = (1 << 31) | 0x0707;

    std::uintptr_t delta = DMA_BUS_BASE - Mailbox::PERIPH_BUS_BASE;
    volatile uint8_t *dma_base = dma_config_.peripheral_base_virtual + delta;
    volatile struct DMAregs *DMA0 = reinterpret_cast<volatile struct DMAregs *>(dma_base);
    DMA0->CS = 1 << 31;
    DMA0->CONBLK_AD = 0;
    DMA0->TI = 0;
    DMA0->CONBLK_AD = static_cast<uint32_t>(instr_page.b);
    DMA0->CS = (1 << 0) | (255 << 16);
}

void WsprRpiBackend::setup_dma()
{
    mailbox.open();
    get_plld();

    uint32_t base = Mailbox::discoverPeripheralBase();
    dma_config_.peripheral_base_virtual = ::mailbox.mapMem(
        base,
        Mailbox::PAGE_SIZE * NUM_PAGES);

    dma_config_.orig_gp0ctl = access_bus_address(CM_GP0CTL_BUS);
    dma_config_.orig_gp0div = access_bus_address(CM_GP0DIV_BUS);
    dma_config_.orig_pwm_ctl = access_bus_address(PWM_BUS_BASE + 0x00);
    dma_config_.orig_pwm_sta = access_bus_address(PWM_BUS_BASE + 0x04);
    dma_config_.orig_pwm_rng1 = access_bus_address(PWM_BUS_BASE + 0x10);
    dma_config_.orig_pwm_rng2 = access_bus_address(PWM_BUS_BASE + 0x20);
    dma_config_.orig_pwm_fifocfg = access_bus_address(PWM_BUS_BASE + 0x08);

    constexpr int kMaxAttempts = 3;
    int attempts = 0;
    while (true)
    {
        try
        {
            MailboxMemoryPool pool(1025);
            break;
        }
        catch (const std::system_error &e)
        {
            if (e.code().value() == ETIMEDOUT)
            {
                std::ostringstream oss;
                oss << attempts
                    << ") allocating memory pool, retrying.";
                owner_.backendFireTransmitCallback(
                    static_cast<int>(WsprTransmitter::TransmissionCallbackEvent::LOGGING),
                    static_cast<int>(WsprTransmitter::LogLevel::DEBUG),
                    oss.str(),
                    0.0);

                if (++attempts >= kMaxAttempts)
                    throw std::runtime_error(
                        "Mailbox::setup_dma() Too many mailbox timeouts, "
                        "giving up");

                try
                {
                    ::mailbox.close();
                }
                catch (...)
                {
                }
                owner_.backendThrowIfStopRequested("waiting to reopen mailbox");
                (void)owner_.backendWaitInterruptableFor(std::chrono::milliseconds(50));
                owner_.backendThrowIfStopRequested("waiting to reopen mailbox");
                ::mailbox.open();
            }
        }
        catch (...)
        {
            throw;
        }
    }

    create_dma_pages(const_page_, instr_page_, instructions_);
    dma_setup_done_ = true;

    uint32_t div_reg = static_cast<uint32_t>(
        access_bus_address(CLK_BUS_BASE + 41 * 4));
    uint32_t divisor = (div_reg >> 12) & 0xFFF;

    if (divisor == 0)
    {
        throw std::runtime_error(
            "setup_dma(): PWM clock divisor read back as 0 (bad register read/mapping).");
    }

    pwm_clock_init_ = dma_config_.plld_clock_frequency / double(divisor);

    if (!std::isfinite(pwm_clock_init_) || pwm_clock_init_ < 1e6 || pwm_clock_init_ > 2e9)
    {
        throw std::runtime_error(
            "setup_dma(): PWM clock computed out of range (bad divisor/readback).");
    }

    {
        std::ostringstream oss;
        oss << "PWM div reg=0x" << std::hex << div_reg << std::dec
            << " divisor=" << divisor
            << " pwm_clock_init_=" << std::fixed << std::setprecision(3)
            << pwm_clock_init_;
        owner_.backendFireTransmitCallback(
            static_cast<int>(WsprTransmitter::TransmissionCallbackEvent::LOGGING),
            static_cast<int>(WsprTransmitter::LogLevel::DEBUG),
            oss.str(),
            0.0);
    }

    {
        std::ostringstream oss;
        oss << "Actual PWM clock = "
            << std::fixed << std::setprecision(0)
            << pwm_clock_init_
            << " Hz";
        owner_.backendFireTransmitCallback(
            static_cast<int>(WsprTransmitter::TransmissionCallbackEvent::LOGGING),
            static_cast<int>(WsprTransmitter::LogLevel::DEBUG),
            oss.str(),
            0.0);
    }
}

void WsprRpiBackend::setup_dma_freq_table(double &center_freq_actual)
{
    double div_lo = bit_trunc(
                        dma_config_.plld_clock_frequency /
                            (owner_.backendFrequency() - 1.5 * owner_.backendToneSpacing()),
                        -12) +
                    std::pow(2.0, -12);
    double div_hi = bit_trunc(
        dma_config_.plld_clock_frequency /
            (owner_.backendFrequency() + 1.5 * owner_.backendToneSpacing()),
        -12);

    if (std::floor(div_lo) != std::floor(div_hi))
    {
        center_freq_actual =
            dma_config_.plld_clock_frequency / std::floor(div_lo) -
            1.6 * owner_.backendToneSpacing();
        if (owner_.backendFrequency() != 0.0)
        {
            std::stringstream temp;
            temp << "Center frequency has been changed to "
                 << WsprTransmitter::formatFrequencyMHz(center_freq_actual)
                 << " MHz";
            std::ostringstream oss;
            oss << temp.str()
                << " because of hardware limitations.";
            owner_.backendFireTransmitCallback(
                static_cast<int>(WsprTransmitter::TransmissionCallbackEvent::LOGGING),
                static_cast<int>(WsprTransmitter::LogLevel::DEBUG),
                oss.str(),
                0.0);
        }
    }

    double tone0_freq = center_freq_actual - 1.5 * owner_.backendToneSpacing();
    std::vector<std::uint32_t> tuning_word(1024);

    for (int i = 0; i < 8; i++)
    {
        double tone_freq = tone0_freq + (i >> 1) * owner_.backendToneSpacing();
        double div = bit_trunc(dma_config_.plld_clock_frequency / tone_freq, -12);

        if (i % 2 == 0)
        {
            div += std::pow(2.0, -12);
        }

        tuning_word[i] = static_cast<std::uint32_t>(div * std::pow(2.0, 12));
    }

    for (int i = 8; i < 1024; i++)
    {
        double div = 500 + i;
        tuning_word[i] = static_cast<std::uint32_t>(div * std::pow(2.0, 12));
    }

    for (int i = 0; i < 1024; i++)
    {
        reinterpret_cast<std::uint32_t *>(const_page_.v)[i] =
            (0x5Au << 24) + tuning_word[i];

        if ((i % 2 == 0) && (i < 8))
        {
            assert((tuning_word[i] & (~0xFFFu)) == (tuning_word[i + 1] & (~0xFFFu)));
        }
    }
}
