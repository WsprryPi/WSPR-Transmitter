/**
 * @file wspr_transmit.hpp
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

#ifndef WSPR_TRANSMIT_HPP
#define WSPR_TRANSMIT_HPP

// C++ standard library headers
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <variant>
#include <vector>

// POSIX and system headers
#include <sys/time.h> // for struct timeval

// Project headers
#include "wspr_message.hpp"

/**
 * @class WsprTransmitter
 * @brief Encapsulates configuration and DMA-driven transmission of WSPR
 *        signals.
 *
 * @details
 *   The WsprTransmitter class provides a full interface for setting up and
 *   executing Weak Signal Propagation Reporter (WSPR) transmissions on a
 *   Raspberry Pi. It handles:
 *     - Configuration of RF frequency, power level, PPM calibration, and
 *       message parameters via configure().
 *       parameters via configure().
 *     - Low‑level mailbox allocation, peripheral memory mapping, and DMA/PWM
 *       initialization for precise symbol timing.
 *     - Start/stop of transmission loops (tone mode or symbol mode).
 *     - Dynamic updates of PPM correction and DMA frequency tables.
 *     - Safe teardown of DMA, PWM, and mailbox resources (idempotent).
 *
 *   Designed for global instantiation and thread‑safe operation, this class
 *   abstracts the complexities of hardware interaction, allowing higher‑level
 *   code to transmit WSPR messages with minimal boilerplate.
 */
class WsprTransmitter
{
public:
    /**
     * @enum State
     * @brief High-level transmission state for the transmitter.
     *
     * @details
     *   This describes whether the transmitter is available to run, idle, or
     *   actively emitting RF.
     *
     * Invariants:
     * - requestStopTx() never transitions to DISABLED.
     * - Stopping TX preserves hardware readiness.
     */
    enum class State
    {
        /** @brief Not enabled to transmit. */
        DISABLED,

        /** @brief Enabled to transmit and idle. */
        ENABLED,

        /** @brief Actively transmitting. */
        TRANSMITTING,

        /** @brief Recovering from a watchdog stall. */
        RECOVERING,

        /** @brief Transmission finished (one-shot or tone). */
        COMPLETE,

        /** @brief Transmission was cancelled by request. */
        CANCELLED,

        /** @brief Reserved for future fault handling. */
        HUNG
    };

    /**
     * @brief Convert a State to a constant string.
     *
     * @param state State value to convert.
     * @return A constant C string describing the state.
     */
    constexpr const char *stateToString(State state) noexcept
    {
        switch (state)
        {
        case State::DISABLED:
            return "DISABLED";
        case State::ENABLED:
            return "ENABLED";
        case State::TRANSMITTING:
            return "TRANSMITTING";
        case State::RECOVERING:
            return "RECOVERING";
        case State::COMPLETE:
            return "COMPLETE";
        case State::CANCELLED:
            return "CANCELLED";
        case State::HUNG:
            return "HUNG";
        default:
            return "UNKNOWN";
        }
    }

    /**
     * @enum LogLevel
     * @brief Log level for callback messages.
     */
    enum class LogLevel
    {
        DEBUG,
        INFO,
        WARN,
        ERROR,
        FATAL
    };

    /**
     * @brief Convert a State to a lowercase std::string.
     *
     * @param state State value to convert.
     * @return Lowercase string describing the state.
     */
    std::string stateToStringLower(State state);

    /**
     * @brief Constructs a WSPR transmitter with default settings.
     *
     * This is for global constructions, parameters are set via
     * configure().
     */
    WsprTransmitter();

    /**
     * @brief Destroys the WSPR transmitter.
     *
     * Cleans up any allocated resources and stops
     * any running transmission threads.
     */
    ~WsprTransmitter();

    /**
     * @brief Deleted copy constructor.
     *
     * @details
     *   Copying a WsprTransmitter instance is disallowed to prevent multiple
     *   objects from owning the same hardware, DMA resources, or background
     *   threads.
     */
    WsprTransmitter(WsprTransmitter const &) = delete;

    /**
     * @brief Deleted copy assignment operator.
     *
     * @details
     *   Assignment is disabled to avoid transferring ownership of active
     *   hardware mappings or thread state between instances.
     */
    WsprTransmitter &operator=(WsprTransmitter const &) = delete;

    /**
     * @brief Deleted move constructor.
     *
     * @details
     *   Moving is disallowed to ensure the transmitter remains at a stable
     *   address for DMA structures, callbacks, and global references.
     */
    WsprTransmitter(WsprTransmitter &&) = delete;

    /**
     * @brief Deleted move assignment operator.
     *
     * @details
     *   Move assignment is disabled to prevent partial transfer of ownership
     *   of hardware resources and internal synchronization primitives.
     */
    WsprTransmitter &operator=(WsprTransmitter &&) = delete;

    /**
     * @brief Signature for user-provided transmission callbacks.
     *
     * This callback receives either a message string or a frequency value,
     * allowing the user to handle both human-readable messages and numeric
     * data.
     *
     * @param arg A variant containing either a std::string or a double value.
     *            The string may carry a descriptive message, while the double
     *            represents a frequency in Hz (or another unit depending
     *            on context).
     */
    /**
     * @brief Identifies which transmission callback event is being reported.
     */
    enum class TransmissionCallbackEvent
    {
        STARTING,
        COMPLETE,
        LOGGING
    };

    /**
     * @brief Signature for user-provided transmission callback.
     *
     * @param event Indicates whether the callback is for transmission start
     *              or completion.
     * @param level Log level for the message.
     * @param msg   Descriptor string for the transmission; may be empty.
     * @param value For STARTING, the active transmit frequency in Hz.
     *              For COMPLETE, the elapsed transmission time in seconds.
     *              For LOGGING, this value is ignored.
     */
    using TransmissionCallback =
        std::function<void(TransmissionCallbackEvent event,
                           LogLevel level,
                           const std::string &msg,
                           double value)>;

     /**
      * @brief Install an optional callback for transmission notifications.
      *
      * @param[in] cb
      *   Called asynchronously when a transmission starts and when it
      *   completes. The first argument identifies which event is being
      *   reported. If null, no notifications are made.
      */
    void setTransmissionCallbacks(TransmissionCallback cb = {});

    /**
     * @brief Format a frequency in MHz using the transmitter's display rules.
     *
     * @details
     *   This helper ensures callbacks and internal logs format frequencies the
     *   same way, so debug and release builds display identical values.
     *
     * @param frequency_hz Frequency in Hz.
     * @return Frequency formatted in MHz with six digits after the decimal.
     */
    static std::string formatFrequencyMHz(double frequency_hz);

    /**
     * @brief Configure transmitter parameters.
     *
     * @details
     *   Sets frequency, power level, PPM calibration, and message parameters.
     *   This does not start the scheduler or begin transmitting.
     *
     * @details Performs the following sequence:
     *   1. Set the desired RF frequency and power level.
     *   2. Populate WSPR symbol data if transmitting a message.
     *   3. Determine WSPR mode (2‑tone or 15‑tone) and symbol timing.
     *   4. Optionally apply a random frequency offset to spread spectral load.
     *   5. Initialize DMA and mailbox resources.
     *   6. Apply the specified PPM calibration to the PLLD clock.
     *   7. Rebuild the DMA frequency table with the new PPM‑corrected clock.
     *   8. Update the actual center frequency after any hardware adjustments.
     *
     * @param[in] frequency    Target RF frequency in Hz.
     * @param[in] power        Transmit power index (0‑n).
     * @param[in] ppm          Parts-per-million correction to apply (for
     *                          example +11.135).
     * @param[in] callsign     Optional callsign for WSPR message.
     * @param[in] grid_square  Optional Maidenhead grid locator.
     * @param[in] power_dbm    dBm value for WSPR message (ignored if tone).
     * @param[in] use_offset   True to apply a small random offset within band.
     */
    void configure(

        double frequency,
        int power,
        double ppm,
        std::string_view call_sign = {},
        std::string_view grid_square = {},
        int power_dbm = 0,
        bool use_offset = false);

    /**
     * @brief Rebuild the DMA tuning‐word table with a fresh PPM correction.
     *
     * @param ppm_new The new parts‑per‑million offset (e.g. +11.135).
     * @throws std::runtime_error if peripherals aren’t mapped.
     */
    void applyPpmCorrection(double ppm_new);

    /**
     * @brief Configure POSIX scheduling policy and priority for future
     *        transmissions.
     *
     * @details
     *   This must be called before `startTransmission()` if you need
     *   real-time scheduling.
     *   under the given policy/priority.
     *
     * @param[in] policy
     *   One of the standard POSIX policies (for example SCHED_FIFO,
     *   SCHED_RR, SCHED_OTHER).
     * @param[in] priority
     *   Thread priority (1-99) for real-time policies. Ignored under
     *   SCHED_OTHER.
     */
    void setThreadScheduling(int policy, int priority);

    /**
     * @brief Enable or disable one-shot scheduling for WSPR mode.
     *
     * When enabled, the scheduler will launch exactly one WSPR transmission
     * and then stop without scheduling further windows.
     */
    void setOneShot(bool enable) noexcept;

    /**
     * @brief Enable or disable immediate transmission for WSPR mode.
     *
     * When enabled, WSPR mode bypasses the next-window scheduler and starts
     * immediately (useful for testing).
     */
    void setTransmitNow(bool enable) noexcept;

    /**
     * @brief Request a "soft off".
     *
     * Prevents any new WSPR transmissions from being scheduled while allowing
     * any currently running transmission to continue or be stopped cleanly by
     * requestStopTx()/stopAndJoin().
     */
    void requestSoftOff() noexcept;

    /**
     * @brief Clear a previously requested "soft off".
     */
    void clearSoftOff() noexcept;

    /**
     * @brief Start transmission, either immediately or via the scheduler.
     *
     * @details
     *   If `trans_params_.is_tone == true`, this will spawn the transmit
     *   thread right away (bypassing the scheduler). Otherwise it launches
     *   the background scheduler, which will fire at the next WSPR window
     *   and then spawn the transmit thread.
     *
     * @note This call is non-blocking. In tone mode it returns immediately
     *       after spawning the thread; in WSPR mode it returns immediately
     *       after starting the scheduler thread.
     */
    void startAsync();

    /**
     * @brief Stop scheduler and transmission and release hardware.
     *
     * @details
     *   Cancels the scheduler (if running), requests any in-flight
     *   transmission to stop, waits for threads to exit, and performs
     *   hardware shutdown (DMA/PWM/clocks).
     */
    void shutdown();

    /**
     * @brief Request an in-flight transmission to stop.
     *
     * @details
     *   Sets the internal stop flag, wakes any interruptible waits, and
     *   waits for the transmit thread to exit. After this returns, it is
     *   safe to call configure() or applyPpmCorrection() and then restart
     *   with startAsync().
     */
    void requestStopTx();

    /**
     * @brief Request TX stop without joining the transmit thread.
     *
     * @details
     *   Used by the watchdog thread to avoid blocking inside join() if the
     *   transmit thread is slow to unwind. Recovery will later perform the
     *   full shutdown sequence.
     */
    void requestStopTxNoJoin() noexcept;

    /**
     * @brief Forcefully reset the DMA/PWM/clock hardware sequence.
     *
     * @details
     *   This is a best-effort, non-throwing path used for watchdog recovery.
     *   It can be called even when the transmitter state machine believes TX
     *   is stalled.
     */
    void force_dma_reset_sequence() noexcept;


    /**
     * @brief Stop and wait for the scheduler/transmit threads.
     *
     * @details
     *   Requests stop and joins threads as needed.
     */
    void stopAndJoin();

    /**
     * @brief Returns true if the DMA watchdog detected a stalled DMA engine.
     */
    bool watchdogFaulted() const noexcept;

    /**
     * @brief Clears the DMA watchdog fault latch.
     */
    void clearWatchdogFault() noexcept;

    /**
     * @brief Enable or disable automatic recovery after a DMA watchdog stall.
     *
     * @details
     *   When enabled, the watchdog thread will request a full hardware reset
     *   (DMA/PWM/clock teardown and re-init) and will restart the scheduler
     *   using the last configured parameters.
     *
     *   Recovery runs on a dedicated internal worker thread so the watchdog
     *   can exit promptly without risking deadlocks.
     *
     * @param enable True to enable automatic recovery.
     */
    void setWatchdogAutoRecover(bool enable) noexcept;

    /**
     * @brief Returns true if watchdog auto-recovery is enabled.
     */
    bool watchdogAutoRecoverEnabled() const noexcept;

    /**
     * @brief Attempt to recover from a latched watchdog fault immediately.
     *
     * @details
     *   This is a synchronous recovery helper. It stops the scheduler and
     *   transmit thread, resets DMA/PWM/clock state, reinitializes DMA state
     *   with the last configured parameters, clears the watchdog fault latch,
     *   and restarts scheduling via startAsync().
     *
     * @return True if recovery succeeded, false otherwise.
     */
    bool recoverFromWatchdogFault();
    /**
     * @brief Get the current transmission state.
     *
     * @details Returns a value indicating if the system is transmitting
     * in any way.
     *
     * @return The current transmitter state.
     */
    State getState() const noexcept;

    /**
     * @brief Prints current transmission parameters and encoded WSPR symbols.
     *
     * @details Displays the configured WSPR parameters including frequency,
     * power, mode, tone/test settings, and symbol timing. Also prints all
     * WSPR symbols as integer values, grouped for readability.
     *
     * This function is useful for debugging and verifying that all transmission
     * settings and symbol sequences are correctly populated before
     * transmission.
     */
    void dumpParameters();

private:
    /**
     * @brief Start the DMA watchdog thread.
     *
     * @details
     *   Launches the background watchdog that monitors DMA progress and
     *   detects stalled or non-advancing control blocks during transmission.
     *   This function is idempotent if the watchdog is already running.
     */
    void start_watchdog();

    /**
     * @brief Stop the DMA watchdog thread.
     *
     * @details
     *   Requests the watchdog thread to stop and waits for it to exit
     *   cleanly before returning.
     */
    void stop_watchdog();

    /**
     * @brief Request a recovery cycle after a watchdog fault.
     *
     * @details
     *   Safe to call from the watchdog thread. This only signals the recovery
     *   worker and returns immediately.
     */
    void request_watchdog_recovery() noexcept;

    /**
     * @brief Internal recovery worker loop.
     */
    void recovery_worker();

    /**
     * @brief Core recovery implementation guarded by recovery_mtx_.
     */
    bool recover_from_watchdog_fault_locked();

    /**
     * @brief Background thread used to monitor DMA progress.
     *
     * @details
     *   This thread periodically checks the active DMA control block address
     *   to detect stalls or lack of forward progress during transmission.
     */
    std::thread watchdog_thread_{};

    /**
     * @brief Stop flag for the watchdog thread.
     *
     * @details
     *   When set to true, the watchdog thread will exit its monitoring loop
     *   and terminate cleanly.
     */
    std::atomic<bool> watchdog_stop_{true};

    /**
     * @brief Latched watchdog fault indicator.
     *
     * @details
     *   Set when the watchdog detects that the DMA engine has stopped making
     *   forward progress. This flag remains set until cleared explicitly.
     */
    std::atomic<bool> watchdog_faulted_{false};

    /**
     * @brief Enable automatic watchdog recovery.
     */
    std::atomic<bool> watchdog_auto_recover_{true};

    /**
     * @brief Stop flag for the recovery worker thread.
     */
    std::atomic<bool> recovery_stop_{false};

    /**
     * @brief Indicates a recovery cycle has been requested.
     */
    std::atomic<bool> recovery_pending_{false};

    /**
     * @brief True while a recovery cycle is actively running.
     */
    std::atomic<bool> recovery_in_progress_{false};

    /**
     * @brief Rate limiting window for watchdog recovery.
     */
    static constexpr auto kRecoveryWindow =
        std::chrono::minutes(10);

    /**
     * @brief Maximum number of recoveries permitted within the window.
     */
    static constexpr std::size_t kMaxRecoveriesInWindow = 3;

    /**
     * @brief Minimum time between recovery attempts.
     */
    static constexpr auto kMinRecoveryInterval =
        std::chrono::seconds(30);

    /**
     * @brief Mutex guarding recovery rate limit state.
     */
    mutable std::mutex recovery_rate_mtx_{};

    /**
     * @brief Timestamps of recent recovery attempts.
     */
    std::deque<std::chrono::steady_clock::time_point> recovery_attempts_{};

    /**
     * @brief Next time a rate-limited recovery attempt may run.
     */
    std::chrono::steady_clock::time_point recovery_defer_until_{};

    /**
     * @brief State to restore after successful recovery.
     */
    State post_recovery_state_{State::ENABLED};

    /**
     * @brief Worker thread that performs DMA/PWM/clock recovery.
     */
    std::thread recovery_thread_{};

    /**
     * @brief Synchronization for the recovery worker wait loop.
     */
    std::mutex recovery_wait_mtx_;
    std::condition_variable recovery_cv_;

    /**
     * @brief Guards the core recovery sequence against concurrent callers.
     */
    std::mutex recovery_mtx_;

    /**
     * @brief Last observed DMA control block address.
     *
     * @details
     *   Used by the watchdog to determine whether the DMA engine is advancing
     *   through the control block ring.
     */
    std::atomic<std::uint32_t> watchdog_last_conblk_{0};

    /**
     * @brief Timestamp of the last DMA control block change.
     *
     * @details
     *   Stored as a steady-clock tick count representing when the watchdog
     *   last observed progress in the DMA control block pointer.
     */
    std::atomic<std::chrono::steady_clock::time_point::rep>
        watchdog_last_change_ns_{0};

    /**
     * @brief CPU core affinity for the transmit thread.
     *
     * @details
     *   Used to bind the transmit thread to a specific CPU core in order to
     *   reduce scheduling jitter during tight timing loops.
     */
    int tx_cpu_{0};

    /**
     * @brief CPU core affinity for the watchdog thread.
     *
     * @details
     *   Separating the watchdog from the transmit core reduces interference
     *   with real-time transmission timing.
     */
    int watchdog_cpu_{1};

    /**
     * @brief Busy-wait tail duration in nanoseconds.
     *
     * @details
     *   During tight absolute sleeps, the thread will switch to a spin-wait
     *   for the final portion of the delay to reduce wake-up latency and
     *   improve symbol boundary accuracy.
     */
    std::int64_t spin_ns_{200'000};

    /**
     * @brief Invoked when a transmission starts or completes.
     *
     * This callback is fired asynchronously from the transmit thread to
     * notify the user that a transmission has started or completed.
     * Users can assign a function via `setTransmissionCallbacks()` to
     * perform setup, logging, or cleanup work tied to these events.
     */
    TransmissionCallback on_transmit_cb_{};

    /**
     * @brief Background thread for carrying out the transmission.
     *
     * Launched by startTransmission() and joined by
     * join_transmission().
     */
    std::thread tx_thread_;

    /**
     * @brief Guards tx_thread_ lifecycle against stop/scheduler races.
     *
     * @details The scheduler thread can finish a transmission and quickly
     * attempt to start the next. The owning thread may call stopAndJoin()
     * right after the end callback fires. This mutex ensures that joining
     * and launching tx_thread_ cannot interleave in a way that creates an
     * extra transmission or a stuck join.
     */
    std::mutex tx_thread_mtx_;

    /**
     * @brief POSIX scheduling policy for the transmission thread.
     *
     * One of SCHED_FIFO, SCHED_RR, or SCHED_OTHER.
     */
    int thread_policy_ = SCHED_OTHER;

    /**
     * @brief Scheduling priority for the transmission thread.
     *
     * Valid range is 1–99 for real‑time policies; ignored by
     * SCHED_OTHER.
     */
    int thread_priority_ = 0;

    /**
     * @brief Flag indicating that a stop request has been issued.
     *
     * When true, loops in transmit() and transmit_symbol() will
     * exit at the next interruption point.
     */
    std::atomic<bool> stop_requested_{false};

    /**
     * @brief Flag indicating that new transmissions must not be scheduled.
     *
     * When set, the scheduler loop will stop launching new transmissions.
     * This does not stop any currently running transmit thread.
     */
    std::atomic<bool> soft_off_{false};

    /**
     * @brief Flag indicating that the scheduler should run exactly one
     *        transmission and then stop.
     */
    std::atomic<bool> one_shot_{false};

    /**
     * @brief Flag indicating that the next transmission should start
     *        immediately.
     */
    std::atomic<bool> transmit_now_{false};

    /**
     * @brief Optional external stop flag.
     *
     * When set to a non-null pointer, shouldStop() will also consider the
     * external flag value.
     */
    const std::atomic<bool> *external_stop_flag_{nullptr};

    /**
     * @brief Aggregate internal and external stop requests.
     *
     * @return true if either stop_requested_ or the external termination
     *         flag (if provided) is set.
     */
    bool shouldStop() const noexcept;

    /**
     * @brief Wait for the given duration unless a stop is requested.
     *
     * @param duration Duration to wait.
     * @return true if the full duration elapsed, false if interrupted.
     */
    bool waitInterruptableFor(std::chrono::nanoseconds duration);

    /**
     * @brief Sleep to an absolute clock deadline unless a stop is requested.
     *
     * @param clk_id Clock used for the absolute deadline.
     * @param ts_target Absolute deadline for the sleep.
     * @param spin_ns Busy-wait tail in nanoseconds.
     * @return true if the deadline was reached, false if interrupted.
     */
    bool sleepUntilAbsTightInterruptible(
        clockid_t clk_id,
        const timespec &ts_target,
        int64_t spin_ns);

    /**
     * @brief Throw if a stop has been requested.
     *
     * @param context Short context string for diagnostics.
     */
    void throwIfStopRequested(const char *context);

    /**
     * @brief Condition variable used to wake the transmission thread.
     *
     * requestStopTx() calls notify_all() on this to unblock
     * any waits so the thread can observe stop_requested_.
     */
    std::condition_variable stop_cv_;

    /**
     * @brief Mutex paired with stop_cv_.
     *
     * Used to implement interruptible waits that can be woken immediately
     * when requestStopTx() is called.
     */
    mutable std::mutex stop_mtx_;

    /**
     * @brief Stores the current transmission state.
     *
     * True when transmit_on() is called, false when transmit_off() or
     * disable_clock() is called.
     */
    std::atomic<State> state_{State::DISABLED};

    /**
     * @brief Scheduled wall-clock start time for windowed WSPR transmissions.
     *
     * @details
     *   Stored as nanoseconds since the Unix epoch using CLOCK_REALTIME. The
     *   scheduler sets this just before spawning the TX thread so the TX thread
     *   can align the first symbol to the exact WSPR window boundary. A value
     *   of 0 means "start immediately".
     */
    std::atomic<std::int64_t> scheduled_start_rt_ns_{0};

    /**
     * @brief Mutex accompanying stop_cv_ for coordinated waits.
     */
    std::mutex stop_mutex_;

    /**
     * @brief Global dma setup semaphore.
     *
     * Shows if setup_dma() been run and not yet torn down.
     */
    bool dma_setup_done_{false};

    /**
     * @brief Holds the bus and virtual addresses for a physical memory page.
     *
     * This structure is used to store the mapping between the bus address
     * by the application) of a single
     * page of physical memory.
     *
     * @var PageInfo::b
     *      The bus address of the physical memory page.
     * @var PageInfo::v
     *      The virtual address mapped to the physical memory page.
     */
    struct PageInfo
    {
        std::uintptr_t b; ///< Bus address.
        void *v;          ///< Virtual address.
    };

    /**
     * @brief Page information for the constant memory page.
     *
     * This global variable holds the bus and virtual addresses of the
     * constant memory page,
     * which is used to store fixed data required for DMA operations, such
     * as the tuning words
     * for frequency generation.
     */
    struct PageInfo const_page_;

    /**
     * @brief Page information for the DMA instruction page.
     *
     * This global variable holds the bus and virtual addresses of the DMA
     * instruction page,
     * where DMA control blocks (CBs) are stored. This page is used during
     * the setup and
     * operation of DMA transfers.
     */
    struct PageInfo instr_page_;

    /**
     * @brief Array of page information structures for DMA control blocks.
     *
     * This global array contains the bus and virtual addresses for each
     * page used in the DMA
     * instruction chain. It holds 1024 entries, corresponding to the 1024
     * DMA control blocks used
     * for managing data transfers.
     */
    struct PageInfo instructions_[1024];

    /**
     * @brief Random frequency offset for standard WSPR transmissions.
     *
     * This constant defines the range, in Hertz, for random frequency offsets
     * applied to standard WSPR transmissions. The offset is applied
     * symmetrically
     * around the target frequency, resulting in a random variation of ±80 Hz.
     *
     * This helps distribute transmissions within the WSPR band, reducing the
     * likelihood of overlapping signals.
     *
     * @note This offset is applicable for standard WSPR transmissions
     *       (2-minute cycles).
     *
     * @see WSPR15_RAND_OFFSET
     */
    static constexpr int WSPR_RAND_OFFSET = 80;

    /**
     * @brief Nominal symbol duration for WSPR transmissions.
     *
     * This constant represents the nominal time duration of a WSPR symbol,
     * calculated as 8192 samples divided by a sample rate of 12000 Hz.
     *
     * @details This duration is a key parameter in WSPR transmissions,
     * ensuring the correct timing for symbol generation and encoding.
     *
     * @note Any deviation in sample rate or processing latency could affect
     *       the actual symbol duration.
     */
    static constexpr double WSPR_SYMTIME = 8192.0 / 12000.0;

    /**
     * @brief Actual PWM clock frequency used for symbol timing.
     *
     * This field holds the measured PWM clock rate (in Hz) read back from the
     * hardware after configuring the clock divisor. It ensures precise symbol
     * durations across platforms.
     *
     * @details Calculated as:
     *   pwm_clock_init_ = plld_clock_frequency / divisor
     * where `divisor` is read from the PWM clock divider register. Used in
     * `transmit_symbol()` to convert the desired symbol time (`tsym`) into
     * clock ticks.
     */
    double pwm_clock_init_{0};

    /**
     * @brief Bus base address for GPIO peripheral registers.
     *
     * @details
     *   Base bus address used to access GPIO function select, set, clear,
     *   and level registers through the peripheral mapping.
     */
    static constexpr uint32_t GPIO_BUS_BASE = 0x7E200000;

    /**
     * @brief Bus address of the GPCLK0 control register.
     *
     * @details
     *   Used to enable, disable, and configure the GPCLK0 clock source
     *   driving the PWM hardware.
     */
    static constexpr uint32_t CM_GP0CTL_BUS = 0x7E101070;

    /**
     * @brief Bus address of the GPCLK0 divider register.
     *
     * @details
     *   Holds the integer and fractional divider controlling the effective
     *   GPCLK0 output frequency.
     */
    static constexpr uint32_t CM_GP0DIV_BUS = 0x7E101074;

    /**
     * @brief Bus address of the GPIO pads control register (GPIO 0-27).
     *
     * @details
     *   Controls drive strength, slew rate, and hysteresis for GPIO pins
     *   used by the transmitter.
     */
    static constexpr uint32_t PADS_GPIO_0_27_BUS = 0x7E10002C;

    /**
     * @brief Bus base address for clock manager registers.
     *
     * @details
     *   Base address for the clock control and divider registers used to
     *   configure GPCLK and other peripheral clocks.
     */
    static constexpr uint32_t CLK_BUS_BASE = 0x7E101000;

    /**
     * @brief Bus base address for DMA controller registers.
     *
     * @details
     *   Used to configure and control the DMA engine responsible for
     *   feeding PWM data during transmission.
     */
    static constexpr uint32_t DMA_BUS_BASE = 0x7E007000;

    /**
     * @brief Bus base address for PWM controller registers.
     *
     * @details
     *   Used to configure PWM channels, FIFOs, and ranges for
     *   DMA-driven RF output.
     */
    static constexpr uint32_t PWM_BUS_BASE = 0x7E20C000;

//
// This constant controls how many PWM "clocks" worth of work we try
// to cover per inner-loop iteration in transmit_symbol().
//
// On 32-bit builds, the per-iteration overhead of updating DMA control
// blocks is much higher, and a small nominal value can stretch a
// 110-second WSPR frame into multiple minutes. Use a larger chunk size
// on 32-bit to keep runtime bounded.
//
// The symbol timing math is still driven by n_pwmclk_per_sym, so this
// only changes how frequently we patch the DMA ring.
#if INTPTR_MAX == INT32_MAX
    static constexpr std::uint32_t PWM_CLOCKS_PER_ITER_NOMINAL = 50000;
#else
    static constexpr std::uint32_t PWM_CLOCKS_PER_ITER_NOMINAL = 1000;
#endif

    /**
     * @brief Validate the nominal PWM clocks per iteration.
     *
     * @details
     *   Ensures at compile time that the configured nominal number of PWM
     *   clocks per transmit iteration is valid and non-zero.
     */
    static_assert(
        PWM_CLOCKS_PER_ITER_NOMINAL > 0,
        "PWM_CLOCKS_PER_ITER_NOMINAL must be non-zero.");

    /**
     * @brief GPIO drive strength lookup table.
     *
     * @details
     *   Maps a drive-strength index to the corresponding GPIO drive current
     *   in milliamps. The index is written into the pads control register
     *   to select the desired drive capability.
     */
    static inline constexpr std::array<int, 8> DRIVE_STRENGTH_TABLE = {
        2, 4, 6, 8, 10, 12, 14, 16};

    /**
     * @struct WsprTransmissionParams
     * @brief Aggregate of parameters used for a single transmission run.
     *
     * @details
     *   Holds all derived and user-supplied parameters required to perform
     *   either a tone transmission or an encoded WSPR message transmission.
     *   The contents of this structure are populated by configure() and are
     *   treated as read-only during an active transmission.
     */
    struct WsprTransmissionParams
    {
        /**
         * @brief Number of symbols in a WSPR message.
         */
        static const std::size_t symbol_count = MSG_SIZE;

        /**
         * @brief Encoded WSPR symbol values.
         *
         * @details
         *   Each entry represents a symbol index used to select the
         *   corresponding frequency from the DMA tuning table.
         */
        std::array<uint8_t, symbol_count> symbols;

        /**
         * @brief Callsign used for WSPR message encoding.
         */
        std::string call_sign;

        /**
         * @brief Maidenhead grid square used for WSPR message encoding.
         */
        std::string grid_square;

        /**
         * @brief Transmit power in dBm for WSPR message encoding.
         */
        int power_dbm;

        /**
         * @brief Center RF frequency in Hz.
         */
        double frequency;

        /**
         * @brief Parts-per-million correction applied to the clock.
         */
        double ppm;

        /**
         * @brief True when operating in tone (unmodulated) mode.
         */
        bool is_tone;

        /**
         * @brief Output power level index.
         */
        int power;

        /**
         * @brief Symbol duration in seconds.
         */
        double symtime;

        /**
         * @brief Tone spacing in Hz between adjacent WSPR symbols.
         */
        double tone_spacing;

        /**
         * @brief DMA tuning table frequencies in Hz.
         *
         * @details
         *   This table is populated during configuration and used by the
         *   transmit loop to drive frequency changes with precise timing.
         */
        std::vector<double> dma_table_freq;

        /**
         * @brief Enable randomized in-band frequency offset.
         */
        bool use_offset;

        /**
         * @brief Construct parameters with safe defaults.
         */
        WsprTransmissionParams()
            : symbols{},
              call_sign{},
              grid_square{},
              power_dbm(0),
              frequency(0.0),
              ppm(0.0),
              is_tone(false),
              power(0),
              symtime(0.0),
              tone_spacing(0.0),
              dma_table_freq(1024, 0.0),
              use_offset(false)
        {
        }
    };

    /**
     * @brief Active transmission parameter set.
     *
     * @details
     *   This instance holds the currently configured transmission parameters
     *   used by the scheduler and transmit thread.
     */
    WsprTransmissionParams trans_params_;
    /**
     * @struct DMAConfig
     * @brief Holds DMA and clock configuration state.
     *
     * @details
     *   This structure stores derived clock frequencies and snapshots of
     *   hardware register state that must be preserved and restored when
     *   enabling or disabling DMA-driven transmission.
     */
    struct DMAConfig
    {
        /**
         * @brief Nominal PLLD frequency in Hz.
         *
         * @details
         *   Base PLLD frequency before any runtime correction or divisor
         *   adjustments are applied.
         */
        double plld_nominal_freq;

        /**
         * @brief Effective PLLD clock frequency in Hz.
         *
         * @details
         *   Actual PLLD frequency after applying PPM correction and divider
         *   configuration.
         */
        double plld_clock_frequency;

        /**
         * @brief Virtual base pointer for mapped peripheral registers.
         *
         * @details
         *   Points to the memory-mapped peripheral region used to access
         *   GPIO, PWM, DMA, and clock registers.
         */
        volatile uint8_t *peripheral_base_virtual;

        /**
         * @brief Saved GPCLK0 control register value.
         */
        uint32_t orig_gp0ctl;

        /**
         * @brief Saved GPCLK0 divider register value.
         */
        uint32_t orig_gp0div;

        /**
         * @brief Saved PWM control register value.
         */
        uint32_t orig_pwm_ctl;

        /**
         * @brief Saved PWM status register value.
         */
        uint32_t orig_pwm_sta;

        /**
         * @brief Saved PWM range register for channel 1.
         */
        uint32_t orig_pwm_rng1;

        /**
         * @brief Saved PWM range register for channel 2.
         */
        uint32_t orig_pwm_rng2;

        /**
         * @brief Saved PWM FIFO configuration register value.
         */
        uint32_t orig_pwm_fifocfg;

        /**
         * @brief Construct with default nominal values.
         *
         * @details
         *   Initializes the nominal PLLD frequency and clears all saved
         *   register snapshots.
         */
        DMAConfig()
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
    };

    /**
     * @brief DMA and clock configuration instance.
     *
     * @details
     *   Holds the active DMA and clock configuration for the current
     *   transmitter instance.
     */
    DMAConfig dma_config_;

    /**
     * @struct MailboxStruct
     * @brief Tracks mailbox-managed memory pool state.
     *
     * @details
     *   This structure records the handle and addressing information for
     *   memory allocated through the Raspberry Pi mailbox interface. The
     *   memory pool is used to back DMA control blocks and constant data
     *   required during transmission.
     */
    struct MailboxStruct
    {
        /**
         * @brief Mailbox allocation handle.
         *
         * @details
         *   Identifier returned by the mailbox interface when allocating
         *   contiguous GPU memory.
         */
        uint32_t mem_ref = 0;

        /**
         * @brief Bus address of the allocated memory pool.
         */
        std::uintptr_t bus_addr = 0;

        /**
         * @brief Virtual address of the mapped memory pool.
         *
         * @details
         *   Used by the application to access mailbox-allocated memory.
         */
        volatile uint8_t *virt_addr = nullptr;

        /**
         * @brief Total size of the allocated memory pool in bytes.
         */
        unsigned pool_size = 0;

        /**
         * @brief Number of pages allocated in the pool.
         */
        unsigned pool_cnt = 0;
    };

    /**
     * @brief Mailbox memory pool instance.
     *
     * @details
     *   Holds the active mailbox allocation used by the transmitter.
     */
    MailboxStruct mailbox_struct_;

    /**
     * @struct CB
     * @brief DMA control block structure.
     *
     * @details
     *   Represents a single DMA control block as consumed by the BCM DMA
     *   engine. Fields correspond directly to the hardware-defined control
     *   block layout and must not be reordered.
     */
    struct CB
    {
        /** @brief Transfer information flags. */
        volatile unsigned int TI;

        /** @brief Source bus address. */
        volatile unsigned int SOURCE_AD;

        /** @brief Destination bus address. */
        volatile unsigned int DEST_AD;

        /** @brief Transfer length and 2D stride control. */
        volatile unsigned int TXFR_LEN;

        /** @brief 2D stride configuration. */
        volatile unsigned int STRIDE;

        /** @brief Bus address of the next control block. */
        volatile unsigned int NEXTCONBK;

        /** @brief Reserved field, must be zero. */
        volatile unsigned int RES1;

        /** @brief Reserved field, must be zero. */
        volatile unsigned int RES2;
    };

    /**
     * @struct GPCTL
     * @brief GPCLK control register bitfield layout.
     *
     * @details
     *   Maps directly onto the GPCLK control register. Bit widths and
     *   ordering must match the hardware specification exactly.
     */
    struct GPCTL
    {
        /** @brief Clock source selection. */
        uint32_t SRC : 4;

        /** @brief Clock enable. */
        uint32_t ENAB : 1;

        /** @brief Kill clock output. */
        uint32_t KILL : 1;

        /** @brief Reserved bit. */
        uint32_t : 1;

        /** @brief Clock busy status. */
        uint32_t BUSY : 1;

        /** @brief Output invert control. */
        uint32_t FLIP : 1;

        /** @brief Clock MASH filter setting. */
        uint32_t MASH : 2;

        /** @brief Reserved bits. */
        uint32_t : 13;

        /** @brief Write password field. */
        uint32_t PASSWD : 8;
    };

    /**
     * @brief Compile-time validation of GPCTL layout size.
     *
     * @details
     *   Ensures the GPCTL bitfield maps to a single 32-bit register.
     */
    static_assert(
        sizeof(GPCTL) == 4,
        "GPCTL must be exactly 32 bits.");

    /**
     * @struct DMAregs
     * @brief DMA channel register layout.
     *
     * @details
     *   Represents the memory-mapped registers for a single DMA channel.
     *   Used to control and monitor DMA engine operation.
     */
    struct DMAregs
    {
        /** @brief Control and status register. */
        volatile unsigned int CS;

        /** @brief Current control block address register. */
        volatile unsigned int CONBLK_AD;

        /** @brief Transfer information mirror register. */
        volatile unsigned int TI;

        /** @brief Source address mirror register. */
        volatile unsigned int SOURCE_AD;

        /** @brief Destination address mirror register. */
        volatile unsigned int DEST_AD;

        /** @brief Transfer length mirror register. */
        volatile unsigned int TXFR_LEN;

        /** @brief 2D stride mirror register. */
        volatile unsigned int STRIDE;

        /** @brief Next control block address mirror register. */
        volatile unsigned int NEXTCONBK;

        /** @brief Debug register. */
        volatile unsigned int DEBUG;
    };

    /**
     * @brief Invoke the configured transmission callback.
     *
     * @details
     *   Calls the user-provided callback, if one is installed, passing
     *   the event type, an optional descriptive message, and an associated
     *   value.
     *
     * @param event Identifies whether this is a start or completion
     *              notification.
     * @param msg   Message string describing the transmission.
     * @param value For STARTING, the transmit frequency in Hz.
     *              For COMPLETE, the elapsed transmission time in seconds.
     */
    void fire_transmit_cb(TransmissionCallbackEvent event,
                          LogLevel level,
                          const std::string &msg,
                          double value);

/**
     * @brief Execute the transmission loop.
     *
     * @details
     *   Drives DMA and PWM hardware to emit either a continuous tone or a
     *   full WSPR symbol sequence. This function runs on the transmit
     *   worker thread.
     */
    void transmit();

    /**
     * @brief Join the active transmission thread.
     *
     * @details
     *   Waits for the transmit thread to exit if it is currently running.
     *   Safe to call multiple times.
     */
    void join_transmission();

    /**
     * @brief Clean up DMA-related resources.
     *
     * @details
     *   Tears down DMA state, restores hardware registers, and releases
     *   mailbox-allocated memory used during transmission.
     */
    void dma_cleanup();

    /**
     * @brief Convert a GPIO power level index to milliwatts.
     *
     * @details
     *   Maps a logical power level index to an approximate output power in
     *   milliwatts based on the configured GPIO drive characteristics.
     *
     * @param level Power level index.
     * @return Output power in milliwatts.
     */
    constexpr int get_gpio_power_mw(int level);

    /**
     * @brief Convert milliwatts to dBm.
     *
     * @param mw Power in milliwatts.
     * @return Equivalent power in dBm.
     */
    inline double convert_mw_dbm(double mw);

    /**
     * @brief Access a memory-mapped peripheral register by bus address.
     *
     * @details
     *   Translates a bus address into a reference within the mapped peripheral
     *   region for direct register access.
     *
     * @param bus_addr Bus address of the register.
     * @return Reference to the mapped register value.
     */
    inline volatile int &access_bus_address(std::uintptr_t bus_addr);

    /**
     * @brief Set a bit in a peripheral register.
     *
     * @param base Bus base address of the register.
     * @param bit Bit index to set.
     */
    inline void set_bit_bus_address(std::uintptr_t base, unsigned int bit);

    /**
     * @brief Clear a bit in a peripheral register.
     *
     * @param base Bus base address of the register.
     * @param bit Bit index to clear.
     */
    inline void clear_bit_bus_address(std::uintptr_t base, unsigned int bit);

    /**
     * @brief Read PLLD configuration and derive clock parameters.
     *
     * @details
     *   Queries the hardware PLLD configuration and populates internal
     *   frequency values used for PWM and DMA timing calculations.
     */
    void get_plld();

    /**
     * @brief Allocate a mailbox-managed memory pool.
     *
     * @details
     *   Requests contiguous physical memory via the mailbox interface for
     *   use by DMA control blocks and constant data pages.
     *
     * @param numpages Number of memory pages to allocate.
     */
    void allocate_memory_pool(unsigned numpages);

    /**
     * @brief Acquire a physical memory page from the pool.
     *
     * @details
     *   Retrieves one page of memory from the mailbox pool and returns both
     *   its virtual and bus addresses.
     *
     * @param vAddr Output pointer to receive the virtual address.
     * @param bAddr Output pointer to receive the bus address.
     */
    void get_real_mem_page_from_pool(void **vAddr, void **bAddr);

    /**
     * @brief Release the mailbox-managed memory pool.
     *
     * @details
     *   Frees all memory previously allocated via the mailbox interface and
     *   clears associated tracking state.
     */
    void deallocate_memory_pool();

    /**
     * @brief Disable transmitter hardware in a controlled sequence.
     *
     * @details
     *   Shuts down DMA, PWM, and clock outputs while restoring any modified
     *   register state. The sequence ensures hardware is left in a safe
     *   state.
     */
    void disable_hardware_sequence();

    /**
     * @brief Disable the output clock driving the PWM hardware.
     *
     * @details
     *   Stops the GPCLK source used for RF generation.
     */
    void disable_clock();

    /**
     * @brief Enable RF output for transmission.
     *
     * @details
     *   Activates clocks and GPIO routing required to begin emitting RF.
     */
    void transmit_on();

    /**
     * @brief Disable RF output after transmission.
     *
     * @details
     *   Turns off clocks and GPIO routing to stop RF emission cleanly.
     */
    void transmit_off();

    /**
     * @brief Transmit work corresponding to a single WSPR symbol.
     *
     * @details
     *   Advances the DMA control block ring to emit the waveform segment
     *   associated with one symbol interval. Frequency changes are applied
     *   by updating DMA control blocks in-place while maintaining precise
     *   symbol timing.
     *
     * @param sym_num Sequential symbol number within the transmission.
     * @param tsym Symbol duration in seconds.
     * @param bufPtr Current DMA buffer pointer, updated as blocks advance.
     * @param symbol_index Optional symbol index override.
     */
    void transmit_symbol(
        const std::uint32_t &sym_num,
        const double &tsym,
        std::uint32_t &bufPtr,
        int symbol_index = -1);

    /**
     * @brief Truncate a floating-point value to a given number of LSBs.
     *
     * @details
     *   Used to limit fractional precision when computing tuning words or
     *   divisors to match hardware resolution.
     *
     * @param d Input value.
     * @param lsb Number of least-significant bits to retain.
     * @return Truncated value.
     */
    double bit_trunc(const double &d, const int &lsb);

    /**
     * @brief Create DMA pages and instruction ring.
     *
     * @details
     *   Initializes the constant data page, instruction page, and DMA
     *   control block ring used for DMA-driven PWM output.
     *
     * @param const_page Reference to the constant data page mapping.
     * @param instr_page Reference to the instruction page mapping.
     * @param instructions Array of instruction page mappings.
     */
    void create_dma_pages(
        PageInfo &const_page,
        PageInfo &instr_page,
        PageInfo instructions[]);

    /**
     * @brief Initialize DMA and related hardware.
     *
     * @details
     *   Configures DMA channels, control blocks, PWM, and clock routing
     *   required for DMA-driven RF generation.
     */
    void setup_dma();

    /**
     * @brief Build the DMA frequency table.
     *
     * @details
     *   Populates the tuning-word frequency table based on the configured
     *   center frequency and PPM correction. The realized center frequency
     *   after hardware quantization is returned to the caller.
     *
     * @param center_freq_actual Output value for the realized center
     *        frequency in Hz.
     */
    void setup_dma_freq_table(double &center_freq_actual);

    /**
     * @brief Entry point for the transmit worker thread.
     *
     * @details
     *   Applies thread scheduling, fires start callbacks, executes the
     *   transmission loop, and performs post-transmit cleanup.
     */
    void thread_entry();

    /**
     * @brief Apply configured scheduling policy and priority to the thread.
     *
     * @details
     *   Uses the previously configured POSIX scheduling parameters to
     *   adjust the priority of the calling thread.
     */
    void set_thread_priority();

    /**
     * @class TransmissionScheduler
     * @brief Schedules WSPR message transmissions on time-aligned windows.
     *
     * @details
     *   Runs in a background thread and waits for the next valid WSPR
     *   transmission window. When triggered, it launches the transmit
     *   worker thread on the parent WsprTransmitter instance.
     *
     *   The scheduler supports one-shot operation, immediate transmission,
     *   and soft-off behavior.
     */
    class TransmissionScheduler
    {
    public:
        /**
         * @brief Construct a scheduler bound to a transmitter instance.
         *
         * @param parent Pointer to the owning WsprTransmitter.
         */
        explicit TransmissionScheduler(WsprTransmitter *parent);

        /**
         * @brief Destroy the scheduler.
         *
         * @details
         *   Requests the scheduler thread to stop and waits for it to exit.
         */
        ~TransmissionScheduler();

        /**
         * @brief Start the scheduler thread.
         */
        void start();

        /**
         * @brief Stop the scheduler thread.
         *
         * @details
         *   Requests termination and wakes the scheduler if it is sleeping.
         */
        void stop();

        /**
         * @brief Wake the scheduler from a wait.
         *
         * @details
         *   Used to interrupt the scheduler when configuration or control
         *   state changes.
         */
        void notify() noexcept;

    private:
        /**
         * @brief Parent transmitter instance.
         */
        WsprTransmitter *parent_;

        /**
         * @brief Scheduler worker thread.
         */
        std::thread thread_;

        /**
         * @brief Stop flag for the scheduler thread.
         */
        std::atomic<bool> stop_requested_{false};

        /**
         * @brief Mutex protecting scheduler wait state.
         */
        std::mutex mtx_;

        /**
         * @brief Condition variable used for scheduler waits.
         */
        std::condition_variable cv_;

        /**
         * @brief Compute the next WSPR transmission window.
         *
         * @return Time point of the next scheduler event.
         */
        std::chrono::system_clock::time_point nextEvent() const;

        /**
         * @brief Scheduler main loop.
         */
        void run();
    };

    /**
     * @brief Scheduler instance for this transmitter.
     *
     * @details
     *   Manages window-based transmission timing for message mode.
     */
    TransmissionScheduler scheduler_{this};
};

/**
 * @brief Global WSPR transmitter instance.
 *
 * @details
 *   Declares the project-wide WsprTransmitter object used to configure
 *   and control RF transmission. The instance is defined in a corresponding
 *   translation unit and is intended to be shared across modules.
 */
extern WsprTransmitter wsprTransmitter;

#endif // WSPR_TRANSMIT_HPP
