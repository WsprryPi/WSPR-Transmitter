/**
 * @file wspr_transmit.cpp
 * @brief A class to encapsulate configuration and DMA‑driven transmission of
 *        WSPR signals.
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

// C++ standard library headers
#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

// POSIX and system headers
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>

// Project headers
#include "wspr_transmit.hpp" // Class Declarations
#include "wspr_transmit_backend_rpi.hpp"
#include "wspr_message.hpp"

// Helper classes and functions in anonymous namespace
namespace
{
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

    static inline int64_t diff_ns(const timespec &a, const timespec &b)
    {
        return (a.tv_sec - b.tv_sec) * 1'000'000'000LL + (a.tv_nsec - b.tv_nsec);
    }

    static inline void busy_wait_until(clockid_t clk_id, const timespec &ts_target)
    {
        for (;;)
        {
            timespec now{};
            clock_gettime(clk_id, &now);
            if (diff_ns(now, ts_target) >= 0)
                break;
        }
    }

    static inline timespec add_ns(timespec t, int64_t ns)
    {
        t.tv_sec += ns / 1000000000LL;
        t.tv_nsec += static_cast<long>(ns % 1000000000LL);
        if (t.tv_nsec >= 1000000000L)
        {
            t.tv_sec++;
            t.tv_nsec -= 1000000000L;
        }
        else if (t.tv_nsec < 0)
        {
            t.tv_sec--;
            t.tv_nsec += 1000000000L;
        }
        return t;
    }

} // end anonymous namespace

WsprTransmitter::TransmissionScheduler::TransmissionScheduler(
    WsprTransmitter *parent)
    : parent_{parent}
{
}

WsprTransmitter::TransmissionScheduler::~TransmissionScheduler()
{
    stop();
}

void WsprTransmitter::TransmissionScheduler::start()
{
    if (thread_.joinable())
        return;

    stop_requested_.store(false, std::memory_order_release);
    thread_ = std::thread(&TransmissionScheduler::run, this);
}

void WsprTransmitter::TransmissionScheduler::stop()
{
    {
        std::lock_guard<std::mutex> lk(mtx_);
        stop_requested_.store(true, std::memory_order_release);
    }
    cv_.notify_all();

    if (thread_.joinable() &&
        thread_.get_id() != std::this_thread::get_id())
    {
        thread_.join();
    }
}

void WsprTransmitter::TransmissionScheduler::notify() noexcept
{
    cv_.notify_all();
}

std::chrono::system_clock::time_point
WsprTransmitter::TransmissionScheduler::nextEvent() const
{
    using namespace std::chrono;

    auto now = system_clock::now();
    auto secs = duration_cast<seconds>(now.time_since_epoch()).count();

    const int cycle = 2 * 60;

    auto idx = secs / cycle;

    auto base = idx * cycle;
    seconds target_secs;
    if (secs < base + 1)
    {
        target_secs = seconds{base + 1};
    }
    else
    {
        target_secs = seconds{(idx + 1) * cycle + 1};
    }

    return system_clock::time_point{target_secs};
}

void WsprTransmitter::TransmissionScheduler::run()
{
    std::chrono::system_clock::time_point last_when{};

    while (!stop_requested_.load(std::memory_order_acquire) &&
           !parent_->soft_off_.load(std::memory_order_acquire))
    {
        if (parent_->external_stop_flag_ &&
            parent_->external_stop_flag_->load(std::memory_order_acquire))
        {
            break;
        }

        auto when = nextEvent();

        // Never reuse the same WSPR window twice. This matters most for
        // zero-frequency skip windows, which complete almost immediately at
        // the boundary. Without remembering the last scheduled boundary, the
        // scheduler can loop fast enough to compute the same window again and
        // emit a second completion for the same slot.
        if (last_when.time_since_epoch().count() != 0 &&
            when <= last_when)
        {
            when = last_when + std::chrono::seconds(2 * 60);
        }

        // Be conservative about late or ambiguous scheduling.
        //
        // WSPR frames must start exactly on the window boundary. If
        // the computed boundary is effectively "now" or in the past
        // (for example due to clock adjustments or coarse rounding),
        // do not start late. Skip to the next window instead.
        const auto now_check = std::chrono::system_clock::now();
        constexpr auto kLateTolerance = std::chrono::milliseconds(50);
        if (now_check + kLateTolerance >= when)
        {
            when += std::chrono::seconds(2 * 60);
        }

        // Spawn the TX thread slightly before the window boundary so it
        // can apply affinity/scheduling and then sleep until the exact
        // boundary.
        constexpr auto kLead = std::chrono::seconds(2);

        const auto pre = when - kLead;

        std::unique_lock<std::mutex> lk(mtx_);
        while (!stop_requested_.load(std::memory_order_acquire) &&
               !parent_->soft_off_.load(std::memory_order_acquire) &&
               std::chrono::system_clock::now() < pre)
        {
            cv_.wait_until(
                lk,
                pre,
                [this]
                {
                    return stop_requested_.load(std::memory_order_acquire);
                });
        }

        if (stop_requested_.load(std::memory_order_acquire) ||
            parent_->soft_off_.load(std::memory_order_acquire))
        {
            break;
        }

        // If we missed the boundary, skip this cycle. This prevents
        // "starting late" when the daemon is launched too late or the
        // system is heavily loaded.
        const auto now = std::chrono::system_clock::now();
        if (now > when + kLateTolerance)
        {
            continue;
        }

        // If a stop was requested while we were evaluating timing,
        // do not schedule another transmission.
        if (stop_requested_.load(std::memory_order_acquire) ||
            parent_->soft_off_.load(std::memory_order_acquire))
        {
            break;
        }

        const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            when.time_since_epoch())
                            .count();
        parent_->scheduled_start_rt_ns_.store(
            ns,
            std::memory_order_release);

        // Synchronize with stop()/shutdown() so we don't
        // race a join/start with a shutdown request.
        std::lock_guard<std::mutex> tx_lk(parent_->tx_thread_mtx_);

        if (stop_requested_.load(std::memory_order_acquire) ||
            parent_->shouldStop())
        {
            break;
        }

        // Join any prior TX thread before launching a new one.
        if (parent_->tx_thread_.joinable())
        {
            parent_->tx_thread_.join();
        }

        // If we waited for a prior transmission to finish and are now
        // past the target window, do not start late. Instead, skip to
        // the next computed window.
        const auto now_post_join = std::chrono::system_clock::now();
        if (now_post_join > when + kLateTolerance)
        {
            continue;
        }

        // Clear the parent stop flag only immediately before launch.
        parent_->stop_requested_.store(false, std::memory_order_release);

        parent_->tx_thread_ = std::thread(
            &WsprTransmitter::thread_entry,
            parent_);

        last_when = when;

        if (parent_->one_shot_.load(std::memory_order_acquire))
        {
            break;
        }
    }
}

WsprTransmitter wsprTransmitter;

/* Public Methods */

WsprTransmitter::WsprTransmitter()
{
    const int ncpu = cpu_count();
    tx_cpu_ = clamp_cpu(tx_cpu_, ncpu);

    if (ncpu <= 1)
    {
        spin_ns_ = 0; // or 50'000 if you want a tiny spin
    }
    backend_ = std::make_unique<WsprRpiBackend>(*this);
}

WsprTransmitter::~WsprTransmitter()
{
    shutdown();
    cleanupTransmissionBackend();
}

void WsprTransmitter::setTransmissionCallbacks(TransmissionCallback cb)
{
    on_transmit_cb_ = std::move(cb);
}

std::string WsprTransmitter::formatFrequencyMHz(double frequency_hz)
{
    const auto hz_rounded =
        static_cast<std::int64_t>(std::llround(frequency_hz));

    const double mhz = static_cast<double>(hz_rounded) / 1.0e6;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << mhz;
    return oss.str();
}

void WsprTransmitter::configure(
    double frequency,
    int power,
    double ppm,
    std::string_view call_sign,
    std::string_view grid_square,
    int power_dbm,
    bool use_offset)
{
    // Reconfiguration is only safe when the transmit thread is not actively
    // feeding DMA. If a transmission is in progress, stop it first.
    if (state_.load(std::memory_order_acquire) == State::TRANSMITTING)
    {
        requestStopTx();
    }

    shutdown();
    cleanupTransmissionBackend();

    stop_requested_.store(false);

    trans_params_.call_sign = call_sign;
    trans_params_.grid_square = grid_square;
    trans_params_.power_dbm = power_dbm;
    trans_params_.frequency = frequency;
    trans_params_.ppm = ppm;
    trans_params_.power = power;
    trans_params_.use_offset = use_offset;

    if (!trans_params_.call_sign.empty() && !trans_params_.grid_square.empty() && trans_params_.power_dbm != 0)
    {
        trans_params_.is_tone = false;
        WsprMessage msg(
            trans_params_.call_sign,
            trans_params_.grid_square,
            trans_params_.power_dbm);
        std::copy_n(msg.symbols, msg.size, trans_params_.symbols.begin());
    }
    else
    {
        trans_params_.is_tone = true;
    }

    int offset_freq = 0;
    trans_params_.symtime = WSPR_SYMTIME;
    if (trans_params_.use_offset)
        offset_freq = WSPR_RAND_OFFSET;
    trans_params_.tone_spacing = 1.0 / trans_params_.symtime;

    if (trans_params_.use_offset)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-1.0, 1.0);
        if (trans_params_.frequency != 0.0)
            trans_params_.frequency += dis(gen) * offset_freq;
    }

    if (!trans_params_.is_tone && trans_params_.frequency == 0.0)
    {
        // A zero WSPR frequency designates an intentional skipped window.
        // Do not initialize DMA or mailbox resources for this cycle.
        scheduled_start_rt_ns_.store(0, std::memory_order_release);
        state_.store(State::ENABLED, std::memory_order_release);
        return;
    }

    try
    {
        prepareTransmissionBackend();

        double center_actual = trans_params_.frequency;
        configureTransmissionBackend(center_actual);

        if (trans_params_.frequency != 0.0)
            trans_params_.frequency = center_actual;

        state_.store(State::ENABLED, std::memory_order_release);
    }
    catch (...)
    {
        try
        {
            shutdown();
            cleanupTransmissionBackend();
        }
        catch (...)
        {
        }
        throw;
    }
}

void WsprTransmitter::applyPpmCorrection(double ppm_new)
{
    // Reconfiguration is only safe when the transmit thread is not actively
    // feeding DMA. If a transmission is in progress, stop it first.
    if (state_.load(std::memory_order_acquire) == State::TRANSMITTING)
    {
        requestStopTx();
    }

    if (!trans_params_.is_tone && trans_params_.frequency == 0.0)
    {
        return;
    }

    double center_actual = trans_params_.frequency;
    configureTransmissionBackend(center_actual);
    if (trans_params_.frequency != 0.0)
        trans_params_.frequency = center_actual;
}

void WsprTransmitter::setThreadScheduling(int policy, int priority)
{
    thread_policy_ = policy;
    thread_priority_ = priority;
}

void WsprTransmitter::setOneShot(bool enable) noexcept
{
    one_shot_.store(enable, std::memory_order_release);
}

void WsprTransmitter::setTransmitNow(bool enable) noexcept
{
    transmit_now_.store(enable, std::memory_order_release);
}

void WsprTransmitter::requestSoftOff() noexcept
{
    soft_off_.store(true, std::memory_order_release);
    scheduler_.notify();
}

void WsprTransmitter::clearSoftOff() noexcept
{
    soft_off_.store(false, std::memory_order_release);
}

void WsprTransmitter::startAsync()
{
    stop_requested_.store(false, std::memory_order_release);

    if (!trans_params_.is_tone && trans_params_.frequency == 0.0)
    {
        // Intended skip window. Do not initialize DMA, but still let the
        // normal scheduler/thread path advance to the scheduled window so the
        // caller receives a completion event at the correct time.
        const State prior = state_.load(std::memory_order_acquire);
        if (prior == State::DISABLED || prior == State::COMPLETE ||
            prior == State::CANCELLED)
        {
            state_.store(State::ENABLED, std::memory_order_release);
        }

        const bool immediate = transmit_now_.load(std::memory_order_acquire);
        if (immediate)
        {
            scheduled_start_rt_ns_.store(0, std::memory_order_release);

            std::lock_guard<std::mutex> lk(tx_thread_mtx_);
            if (tx_thread_.joinable())
            {
                tx_thread_.join();
            }

            tx_thread_ = std::thread(&WsprTransmitter::thread_entry, this);
        }
        else
        {
            scheduler_.start();
        }
        return;
    }

    // If the application has requested a soft-off, do not start scheduling.
    if (!trans_params_.is_tone && soft_off_.load(std::memory_order_acquire))
    {
        return;
    }

    // The application may poll getState() immediately after startAsync().
    // Transition to ENABLED here so callers do not misinterpret the initial
    // DISABLED state as an early abort before the TX thread has a chance to
    // run and advance the state machine.
    {
        const State prior = state_.load(std::memory_order_acquire);
        if (prior == State::DISABLED || prior == State::COMPLETE ||
            prior == State::CANCELLED)
        {
            state_.store(State::ENABLED, std::memory_order_release);
        }
    }

    const bool immediate = trans_params_.is_tone ||
                           transmit_now_.load(std::memory_order_acquire);

    if (immediate)
    {
        // For WSPR "--now" runs, align the TX start to the next 50 ms
        // boundary. This emulates the final timer stage used by the normal
        // window scheduler (which sleeps to an absolute CLOCK_REALTIME
        // boundary).
        if (!trans_params_.is_tone &&
            transmit_now_.load(std::memory_order_acquire))
        {
            struct timespec now_rt{};
            ::clock_gettime(CLOCK_REALTIME, &now_rt);

            const std::int64_t now_ns =
                static_cast<std::int64_t>(now_rt.tv_sec) * 1000000000LL +
                static_cast<std::int64_t>(now_rt.tv_nsec);

            constexpr std::int64_t kTickNs = 50000000LL;   // 50 ms
            constexpr std::int64_t kMinLeadNs = 5000000LL; // 5 ms

            std::int64_t next_ns = ((now_ns / kTickNs) + 1) * kTickNs;
            if (next_ns - now_ns < kMinLeadNs)
            {
                next_ns += kTickNs;
            }

            scheduled_start_rt_ns_.store(next_ns, std::memory_order_release);
        }
        else
        {
            scheduled_start_rt_ns_.store(0, std::memory_order_release);
        }

        std::lock_guard<std::mutex> lk(tx_thread_mtx_);
        if (tx_thread_.joinable())
        {
            tx_thread_.join();
        }

        tx_thread_ = std::thread(&WsprTransmitter::thread_entry, this);
        return;
    }

    scheduler_.start();
}

void WsprTransmitter::shutdown()
{
    // Set the stop flag first so a newly spawned transmit thread
    // will abort before it touches DMA/PWM state.
    stop_requested_.store(true, std::memory_order_release);
    stop_cv_.notify_all();

    // Stop the scheduler thread. Note: do not set soft_off_ here.
    //
    // soft_off_ is an application-level "no new scheduling" latch (used
    // for Ctrl-C / graceful shutdown). shutdown() is also used
    // internally during reconfiguration (e.g., configure()), and
    // must not permanently prevent future enableTransmission() calls.
    scheduler_.stop();

    // Join the transmit thread under a mutex so the scheduler cannot
    // race with us and start a new thread while we are joining.
    {
        std::lock_guard<std::mutex> lk(tx_thread_mtx_);
        if (tx_thread_.joinable() &&
            tx_thread_.get_id() != std::this_thread::get_id())
        {
            tx_thread_.join();
        }
    }

    stopFaultMonitoring();

    // Return to DISABLED when the transmitter is shut down, unless a
    // watchdog recovery is in progress or the transmitter is latched HUNG.
    if (!backend_->recoveryInProgress())
    {
        const State prior = state_.load(std::memory_order_acquire);
        if (prior != State::HUNG && prior != State::RECOVERING)
        {
            state_.store(State::DISABLED, std::memory_order_release);
        }
    }
}

void WsprTransmitter::requestStopTx()
{
    stop_requested_.store(true, std::memory_order_release);
    stop_cv_.notify_all();

    // Synchronize with the scheduler so it cannot race a join/start while
    // we are waiting for the transmit thread to unwind.
    {
        std::lock_guard<std::mutex> lk(tx_thread_mtx_);
        if (tx_thread_.joinable() &&
            tx_thread_.get_id() != std::this_thread::get_id())
        {
            tx_thread_.join();
        }
    }
}

void WsprTransmitter::requestStopTxNoJoin() noexcept
{
    stop_requested_.store(true, std::memory_order_release);
    stop_cv_.notify_all();
}

void WsprTransmitter::force_dma_reset_sequence() noexcept
{
    backend_->resetTransmissionOutput();
}


bool WsprTransmitter::watchdogFaulted() const noexcept
{
    return backend_->faulted();
}

void WsprTransmitter::clearWatchdogFault() noexcept
{
    backend_->clearFault();
}

void WsprTransmitter::setWatchdogAutoRecover(bool enable) noexcept
{
    backend_->setAutoRecover(enable);
}

bool WsprTransmitter::watchdogAutoRecoverEnabled() const noexcept
{
    return backend_->autoRecoverEnabled();
}

bool WsprTransmitter::recoverFromWatchdogFault()
{
    return backend_->recoverFromFault();
}

void WsprTransmitter::request_watchdog_recovery() noexcept
{
    backend_->recoverFromFault();
}

void WsprTransmitter::recovery_worker()
{
}

bool WsprTransmitter::recover_from_watchdog_fault_locked()
{
    return backend_->recoverFromFault();
}

void WsprTransmitter::stopAndJoin()
{
    shutdown();
    cleanupTransmissionBackend();
}

WsprTransmitState WsprTransmitter::getState() const noexcept
{
    return state_.load(std::memory_order_acquire);
}

void WsprTransmitter::dumpParameters()
{
    auto log_line =
        [this](const std::string &line)
    {
        fire_transmit_cb(
            TransmissionCallbackEvent::LOGGING,
            LogLevel::DEBUG,
            line,
            0.0);
    };

    std::ostringstream oss;

    oss << "Call Sign:         "
        << (trans_params_.is_tone ? "N/A" : trans_params_.call_sign);
    log_line(oss.str());
    oss.str("");
    oss.clear();

    oss << "Grid Square:       "
        << (trans_params_.is_tone ? "N/A" : trans_params_.grid_square);
    log_line(oss.str());
    oss.str("");
    oss.clear();

    oss << "WSPR Frequency:    "
        << formatFrequencyMHz(trans_params_.frequency)
        << " MHz";
    log_line(oss.str());
    oss.str("");
    oss.clear();

    oss << "GPIO Power:        "
        << std::fixed
        << std::setprecision(1)
        << convert_mw_dbm(getOutputPowerMilliwatts(trans_params_.power))
        << " dBm";
    log_line(oss.str());
    oss.str("");
    oss.clear();

    oss << "Test Tone:         "
        << (trans_params_.is_tone ? "True" : "False");
    log_line(oss.str());
    oss.str("");
    oss.clear();

    oss << "WSPR Symbol Time:  "
        << (trans_params_.is_tone
                ? "N/A"
                : (std::to_string(trans_params_.symtime) + " s"));
    log_line(oss.str());
    oss.str("");
    oss.clear();

    oss << "WSPR Tone Spacing: "
        << (trans_params_.is_tone
                ? "N/A"
                : (std::to_string(trans_params_.tone_spacing) + " Hz"));
    log_line(oss.str());
    oss.str("");
    oss.clear();

    oss << "DMA Table Size:    "
        << 1024;
    log_line(oss.str());
    oss.str("");
    oss.clear();

    if (trans_params_.is_tone)
    {
        log_line("WSPR Symbols:      N/A");
    }
    else
    {
        log_line("WSPR Symbols:");

        const int symbol_count =
            static_cast<int>(trans_params_.symbols.size());

        std::string line;
        line.reserve(128);

        for (int i = 0; i < symbol_count; ++i)
        {
            line += std::to_string(
                static_cast<int>(trans_params_.symbols[i]));

            if (i < symbol_count - 1)
            {
                line += ", ";
            }

            if ((i + 1) % 18 == 0 || i == symbol_count - 1)
            {
                log_line(line);
                line.clear();
            }
        }
    }
}

/* Private Methods */

inline void WsprTransmitter::fire_transmit_cb(
    TransmissionCallbackEvent event,
    LogLevel level,
    const std::string &msg,
    double value)
{
    if (on_transmit_cb_)
    {
        std::thread([cb = on_transmit_cb_, event, level, msg, value]()
                    { cb(event, level, msg, value); })
            .detach();
    }
}

WsprTransmitState WsprTransmitter::backendStateValue() const noexcept
{
    return state_.load(std::memory_order_acquire);
}

void WsprTransmitter::backendSetStateValue(WsprTransmitState state) noexcept
{
    state_.store(state, std::memory_order_release);
}

bool WsprTransmitter::backendShouldStop() const noexcept
{
    return shouldStop();
}

void WsprTransmitter::backendSignalStopRequest() noexcept
{
    stop_requested_.store(true, std::memory_order_release);
    stop_cv_.notify_all();
}

void WsprTransmitter::backendRequestStopTxNoJoin() noexcept
{
    requestStopTxNoJoin();
}

bool WsprTransmitter::backendWaitInterruptableFor(std::chrono::nanoseconds duration)
{
    return waitInterruptableFor(duration);
}

void WsprTransmitter::backendThrowIfStopRequested(const char *context)
{
    throwIfStopRequested(context);
}

void WsprTransmitter::backendFireTransmitCallback(
    WsprTransmissionCallbackEvent event,
    WsprTransmitLogLevel level,
    const std::string &msg,
    double value)
{
    fire_transmit_cb(event, level, msg, value);
}

bool WsprTransmitter::backendRestartCurrentConfiguration()
{
    const double frequency = trans_params_.frequency;
    const int power = trans_params_.power;
    const double ppm = trans_params_.ppm;
    const std::string call_sign = trans_params_.call_sign;
    const std::string grid_square = trans_params_.grid_square;
    const int power_dbm = trans_params_.power_dbm;
    const bool use_offset = trans_params_.use_offset;

    shutdown();
    cleanupTransmissionBackend();
    configure(frequency, power, ppm, call_sign, grid_square, power_dbm,
              use_offset);
    startAsync();
    return true;
}

WsprTransmissionPlan WsprTransmitter::buildTransmissionPlan() const noexcept
{
    return WsprTransmissionPlan{
        trans_params_.frequency,
        trans_params_.tone_spacing,
        trans_params_.power,
        trans_params_.symbols.size()};
}

bool WsprTransmitter::shouldStop() const noexcept
{
    if (stop_requested_.load(std::memory_order_acquire))
        return true;

    const std::atomic<bool> *ext = external_stop_flag_;
    if (ext && ext->load(std::memory_order_acquire))
        return true;

    return false;
}

void WsprTransmitter::startFaultMonitoring()
{
    backend_->startFaultMonitoring();
}

void WsprTransmitter::stopFaultMonitoring()
{
    backend_->stopFaultMonitoring();
}

bool WsprTransmitter::waitInterruptableFor(std::chrono::nanoseconds duration)
{
    std::unique_lock<std::mutex> lk(stop_mtx_);
    const bool interrupted = stop_cv_.wait_for(
        lk,
        duration,
        [this]
        {
            return shouldStop() || soft_off_.load(std::memory_order_acquire);
        });

    return !interrupted;
}

bool WsprTransmitter::sleepUntilAbsTightInterruptible(
    clockid_t clk_id,
    const timespec &ts_target,
    int64_t spin_ns)
{
    if (spin_ns < 0)
    {
        spin_ns = 0;
    }

    for (;;)
    {
        if (shouldStop() || soft_off_.load(std::memory_order_acquire))
        {
            return false;
        }

        timespec now{};
        ::clock_gettime(clk_id, &now);

        const int64_t remaining_ns = diff_ns(ts_target, now);
        if (remaining_ns <= 0)
        {
            break;
        }

        if (remaining_ns > spin_ns)
        {
            const auto sleep_ns =
                std::chrono::nanoseconds{remaining_ns - spin_ns};

            // Use an interruptible condition-variable wait for the bulk
            // of the sleep so requestStopTx() can wake us promptly.
            if (!waitInterruptableFor(sleep_ns))
            {
                return false;
            }
            continue;
        }

        // Final precision tail: busy-wait until the exact deadline.
        while (!shouldStop())
        {
            ::clock_gettime(clk_id, &now);
            if (diff_ns(now, ts_target) >= 0)
            {
                break;
            }
        }
        break;
    }

    return !(shouldStop() || soft_off_.load(std::memory_order_acquire));
}

void WsprTransmitter::throwIfStopRequested(const char *context)
{
    if (!shouldStop() && !soft_off_.load(std::memory_order_acquire))
        return;

    std::string msg = "Stop requested";
    if (context && *context)
    {
        msg += " while ";
        msg += context;
    }
    msg += '.';

    throw std::runtime_error(msg);
}

void WsprTransmitter::transmit()
{
    if (!trans_params_.is_tone && trans_params_.frequency == 0.0)
    {
        const std::int64_t start_rt_ns =
            scheduled_start_rt_ns_.load(std::memory_order_acquire);
        if (start_rt_ns != 0)
        {
            struct timespec start_rt{};
            start_rt.tv_sec = start_rt_ns / 1000000000LL;
            start_rt.tv_nsec = static_cast<long>(start_rt_ns % 1000000000LL);

            if (!sleepUntilAbsTightInterruptible(CLOCK_REALTIME, start_rt, spin_ns_))
            {
                const bool canceled = shouldStop();
                state_.store(canceled ? State::CANCELLED : State::COMPLETE,
                             std::memory_order_release);
                fire_transmit_cb(canceled
                                     ? TransmissionCallbackEvent::COMPLETE
                                     : TransmissionCallbackEvent::SKIPPED,
                                 LogLevel::INFO,
                                 canceled ? "canceled" : "Skipping transmission",
                                 0.0);
                return;
            }
        }

        const bool canceled = shouldStop();
        state_.store(canceled ? State::CANCELLED : State::COMPLETE,
                     std::memory_order_release);
        fire_transmit_cb(canceled
                             ? TransmissionCallbackEvent::COMPLETE
                             : TransmissionCallbackEvent::SKIPPED,
                         LogLevel::INFO,
                         canceled ? "canceled" : "Skipping transmission",
                         0.0);
        return;
    }

    if (shouldStop())
    {
        {
            std::ostringstream oss;
            oss << "transmit() aborted before start.";
            fire_transmit_cb(
                TransmissionCallbackEvent::LOGGING,
                LogLevel::DEBUG,
                oss.str(),
                0.0);
        }

        if (one_shot_.load(std::memory_order_acquire))
        {
            state_.store(State::COMPLETE, std::memory_order_release);
        }

        return;
    }

    // RAII guard that guarantees TX is turned off no matter how we exit.
    struct TxOffGuard
    {
        WsprTransmitter *self;
        bool enabled;

        explicit TxOffGuard(WsprTransmitter *s)
            : self{s}, enabled{true}
        {
        }

        void dismiss()
        {
            enabled = false;
        }

        ~TxOffGuard()
        {
            if (enabled && self)
            {
                self->endTransmissionOutput();
            }
        }
    };

    if (trans_params_.is_tone)
    {
        // Fire callback as close to the first symbol as possible.
        fire_transmit_cb(TransmissionCallbackEvent::STARTING,
                         LogLevel::INFO,
                         "",
                         trans_params_.frequency);

        const auto t0_chrono = std::chrono::steady_clock::now();

        beginTransmissionOutput();
        TxOffGuard tx_guard(this);

        if (!shouldStop())
        {
            emitSymbol(
                0,
                0.0,
                -1);

            if (!shouldStop())
            {
                startFaultMonitoring();
            }
        }

        while (!shouldStop())
        {
            emitSymbol(
                0,
                0.0,
                -1);
        }

        const auto t_end_chrono = std::chrono::steady_clock::now();

        endTransmissionOutput();
        tx_guard.dismiss();

        const bool canceled = shouldStop();

        state_.store(canceled ? State::CANCELLED : State::COMPLETE,
                     std::memory_order_release);

        const double actual =
            std::chrono::duration<double>(t_end_chrono - t0_chrono).count();
        fire_transmit_cb(TransmissionCallbackEvent::COMPLETE,
                         LogLevel::INFO,
                         canceled ? "canceled" : "",
                         actual);
    }
    else
    {
        // Align to the scheduler-provided realtime boundary before starting TX.
        const std::int64_t start_rt_ns =
            scheduled_start_rt_ns_.load(std::memory_order_acquire);
        if (start_rt_ns != 0)
        {
            struct timespec start_rt{};
            start_rt.tv_sec = start_rt_ns / 1000000000LL;
            start_rt.tv_nsec = static_cast<long>(start_rt_ns % 1000000000LL);

            if (!sleepUntilAbsTightInterruptible(CLOCK_REALTIME, start_rt, spin_ns_))
            {
                {
                    std::ostringstream oss;
                    oss << "TX start aborted before window boundary.";
                    fire_transmit_cb(
                        TransmissionCallbackEvent::LOGGING,
                        LogLevel::DEBUG,
                        oss.str(),
                        0.0);
                }

                if (one_shot_.load(std::memory_order_acquire))
                {
                    state_.store(State::COMPLETE, std::memory_order_release);
                }

                return;
            }

            {
                struct timespec now_rt{};
                clock_gettime(CLOCK_REALTIME, &now_rt);
                std::tm tm_rt{};
                gmtime_r(&now_rt.tv_sec, &tm_rt);

                const long usec = now_rt.tv_nsec / 1000;

                {
                    std::ostringstream oss;
                    oss
                        << "TX start realtime = "
                        << std::setw(2) << std::setfill('0') << tm_rt.tm_hour << ":"
                        << std::setw(2) << std::setfill('0') << tm_rt.tm_min << ":"
                        << std::setw(2) << std::setfill('0') << tm_rt.tm_sec << "."
                        << std::setw(6) << std::setfill('0') << usec
                        << std::setfill(' ')
                        << "Z";
                    fire_transmit_cb(
                        TransmissionCallbackEvent::LOGGING,
                        LogLevel::DEBUG,
                        oss.str(),
                        0.0);
                }
            }
        }

        // Fire callback as close to the first symbol as possible.
        fire_transmit_cb(TransmissionCallbackEvent::STARTING, LogLevel::INFO, "", trans_params_.frequency);

        const int symbol_count = static_cast<int>(trans_params_.symbols.size());
        const double symtime = trans_params_.symtime;

        beginTransmissionOutput();
        TxOffGuard tx_guard(this);

        // Anchor symbol timing to monotonic clock AFTER TX is enabled.
        struct timespec t0_ts{};
        clock_gettime(CLOCK_MONOTONIC, &t0_ts);
        auto t0_chrono = std::chrono::steady_clock::now();

        if (::mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        {
            std::ostringstream oss;
            oss << "mlockall failed: "
                << std::strerror(errno);
            fire_transmit_cb(
                TransmissionCallbackEvent::LOGGING,
                LogLevel::DEBUG,
                oss.str(),
                0.0);
        }

        int i = 0;
        if (symbol_count > 0 && !shouldStop())
        {
            emitSymbol(
                static_cast<int>(trans_params_.symbols[0]),
                symtime,
                0);

            if (!shouldStop())
            {
                startFaultMonitoring();
                i = 1;
            }
            else
            {
                i = 1;
            }
        }

        for (; i < symbol_count && !shouldStop(); ++i)
        {
            const int64_t offset_ns =
                static_cast<int64_t>(
                    std::llround(static_cast<double>(i) * symtime * 1e9));

            timespec target = add_ns(t0_ts, offset_ns);

            if (!sleepUntilAbsTightInterruptible(CLOCK_MONOTONIC, target, 200'000))
            {
                break;
            }

            {
                struct timespec now{};
                clock_gettime(CLOCK_MONOTONIC, &now);

                const int64_t late_ns =
                    (now.tv_sec - target.tv_sec) * 1'000'000'000LL +
                    (now.tv_nsec - target.tv_nsec);

                if (late_ns > 1'000'000) // >1 ms late
                {
                    {
                        std::ostringstream oss;
                        oss << "Symbol overrun: "
                            << late_ns / 1e6
                            << " ms late";
                        fire_transmit_cb(
                            TransmissionCallbackEvent::LOGGING,
                            LogLevel::DEBUG,
                            oss.str(),
                            0.0);
                    }
                }
            }
            if (shouldStop())
            {
                break;
            }

            emitSymbol(
                static_cast<int>(trans_params_.symbols[i]),
                symtime,
                i);
        }

        const bool canceled = shouldStop() && (i < symbol_count);

        // Allow the final symbol's queued DMA work to drain before turning off
        // the clock. Without this, we can cut the last symbol short by roughly
        // one symbol period.
        if (!canceled)
        {
            const int64_t end_ns =
                static_cast<int64_t>(std::llround(
                    static_cast<double>(symbol_count) * symtime * 1e9));

            const timespec end_target = add_ns(t0_ts, end_ns);
            (void)sleepUntilAbsTightInterruptible(CLOCK_MONOTONIC, end_target, 200'000);
        }

        // Capture the end time immediately after the symbol-period drain.
        // transmit_off() can take non-trivial time on some platforms, and that
        // shutdown overhead should not be counted against the on-air duration.
        const auto t_end_chrono = std::chrono::steady_clock::now();

        endTransmissionOutput();
        tx_guard.dismiss();

        state_.store(canceled ? State::CANCELLED : State::COMPLETE,
                     std::memory_order_release);

        const double actual =
            std::chrono::duration<double>(t_end_chrono - t0_chrono).count();
        fire_transmit_cb(TransmissionCallbackEvent::COMPLETE, LogLevel::INFO, canceled ? "canceled" : "", actual);
    }
}

void WsprTransmitter::join_transmission()
{
    if (tx_thread_.joinable())
    {
        tx_thread_.join();
    }
}

void WsprTransmitter::cleanupTransmissionBackend()
{
    backend_->cleanupTransmission();
}

int WsprTransmitter::getOutputPowerMilliwatts(int level)
{
    return backend_->getOutputPowerMilliwatts(level);
}

inline double WsprTransmitter::convert_mw_dbm(double mw)
{
    if (mw <= 0.0)
    {
        throw std::domain_error(
            "WsprTransmitter::convert_mw_dbm: Input power (mW) must "
            "be > 0 to compute logarithm");
    }
    return 10.0 * std::log10(mw);
}

void WsprTransmitter::thread_entry()
{
    const int ncpu = cpu_count();

    if (ncpu > 1)
    {
        cpu_set_t cpus;
        CPU_ZERO(&cpus);
        CPU_SET(tx_cpu_, &cpus);

        const int aff_ret =
            pthread_setaffinity_np(pthread_self(),
                                   sizeof(cpus),
                                   &cpus);

        if (aff_ret != 0)
        {
            {
                std::ostringstream oss;
                oss << "thread_entry(): failed to set CPU affinity: "
                    << std::strerror(aff_ret);
                fire_transmit_cb(
                    TransmissionCallbackEvent::LOGGING,
                    LogLevel::DEBUG,
                    oss.str(),
                    0.0);
            }
        }
    }

    try
    {
        set_thread_priority();
    }
    catch (const std::system_error &e)
    {
        throw std::domain_error(
            std::string(
                "WsprTransmitter::thread_entry(): Error setting thread "
                "priority: ") +
            e.what());
    }
    catch (const std::exception &e)
    {
        throw std::domain_error(
            std::string("WsprTransmitter::thread_entry(): Unexpected error: ") + e.what());
    }
    transmit();
}

void WsprTransmitter::set_thread_priority()
{
    sched_param sch{};
    sch.sched_priority = thread_priority_;
    int ret = pthread_setschedparam(pthread_self(), thread_policy_, &sch);

    if (ret != 0)
    {
        throw std::runtime_error(
            std::string("WsprTransmitter::set_thread_priority(): pthread_setschedparam failed: ") +
            std::strerror(ret));
    }
}
void WsprTransmitter::emitSymbol(
    const std::uint32_t &sym_num,
    const double &tsym,
    int symbol_index)
{
    backend_->emitSymbol(
        buildTransmissionPlan(),
        sym_num,
        tsym,
        symbol_index);
}

void WsprTransmitter::prepareTransmissionBackend()
{
    backend_->prepareTransmission();
}

void WsprTransmitter::configureTransmissionBackend(double &center_freq_actual)
{
    backend_->configureTransmission(buildTransmissionPlan(), center_freq_actual);
}

void WsprTransmitter::beginTransmissionOutput()
{
    backend_->beginTransmissionOutput(buildTransmissionPlan());
}

void WsprTransmitter::endTransmissionOutput()
{
    backend_->endTransmissionOutput();
}

std::string WsprTransmitter::stateToStringLower(State state)
{
    std::string s = wsprTransmitStateToString(state);
    std::transform(
        s.begin(),
        s.end(),
        s.begin(),
        [](unsigned char c)
        { return std::tolower(c); });
    return s;
}
