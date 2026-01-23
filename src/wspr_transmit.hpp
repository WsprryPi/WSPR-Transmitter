/**
 * @file wspr_transmit.hpp
 * @brief A class to encapsulate configuration and DMA‑driven transmission of
 *        WSPR signals.
 *
 * Copyright (C) 2025 Lee C. Bussy (@LBussy). All rights reserved.
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

#ifndef _WSPR_TRANSMIT_HPP
#define _WSPR_TRANSMIT_HPP

// Project header
#include "wspr_message.hpp"

// C++ Standard Library
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <variant>
#include <vector>

// POSIX/System headers
#include <sys/time.h> // for struct timeval

/**
 * @class WsprTransmitter
 * @brief Encapsulates configuration and DMA‑driven transmission of WSPR signals.
 *
 * @details
 *   The WsprTransmitter class provides a full interface for setting up and
 *   executing Weak Signal Propagation Reporter (WSPR) transmissions on a
 *   Raspberry Pi. It handles:
 *     - Configuration of RF frequency, power level, PPM calibration, and message
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
     * @brief Transmission state for the transmitter.
     */
    enum class State
    {
        DISABLED,     /**< Not enabled to transmit */
        ENABLED,      /**< Enabled to transmit, idle */
        TRANSMITTING, /**< Actively transmitting */
        HUNG          /**< Reserved for future fault handling */
    };

    static constexpr const char *stateToString(State state) noexcept;
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

    // non‐copyable, non‐movable
    WsprTransmitter(WsprTransmitter const &) = delete;
    WsprTransmitter &operator=(WsprTransmitter const &) = delete;
    WsprTransmitter(WsprTransmitter &&) = delete;
    WsprTransmitter &operator=(WsprTransmitter &&) = delete;

    /**
     * @brief Signature for user-provided transmission callbacks.
     *
     * This callback receives either a message string or a frequency value,
     * allowing the user to handle both human-readable messages and numeric data.
     *
     * @param arg A variant containing either a std::string or a double value.
     *            The string may carry a descriptive message, while the double
     *            represents a frequency in Hz (or another unit depending on context).
     */
    using StartCallback = std::function<void(const std::string & /*msg*/, double /*frequency*/)>;
    using EndCallback = std::function<void(const std::string & /*msg*/, double /*elapsed_secs*/)>;

    /**
     * @brief Install optional callbacks for transmission start/end.
     *
     * @param[in] start_cb
     *   Called on the transmit thread immediately before the first symbol
     *   (or tone) is emitted.  If null, no start notification is made.
     * @param[in] end_cb
     *   Called on the transmit thread immediately after the last WSPR
     *   symbol is sent (but before DMA/PWM are torn down).  If null,
     *   no completion notification is made.
     */
    void setTransmissionCallbacks(StartCallback start_cb = {},
                                  EndCallback end_cb = {});

    /**
     * @brief Configure and start a WSPR transmission.
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
     * @param[in] ppm          Parts‑per‑million correction to apply (e.g. +11.135).
     * @param[in] callsign     Optional callsign for WSPR message.
     * @param[in] grid_square  Optional Maidenhead grid locator.
     * @param[in] power_dbm    dBm value for WSPR message (ignored if tone).
     * @param[in] use_offset   True to apply a small random offset within band.
     *
     * @throws std::runtime_error if DMA setup or mailbox operations fail.
     */

    /**
     * @brief Configure transmitter parameters.
     *
     * @details
     *   Sets frequency, power level, PPM calibration, and message parameters.
     *   This does not start the scheduler or begin transmitting.
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
     * @brief Configure POSIX scheduling policy & priority for future transmissions.
     *
     * @details
     *   This must be called _before_ `startTransmission()` if you need real-time
     *   scheduling.  The next call to `startTransmission()` will launch its thread
     *   under the given policy/priority.
     *
     * @param[in] policy
     *   One of the standard POSIX policies (e.g. SCHED_FIFO, SCHED_RR, SCHED_OTHER).
     * @param[in] priority
     *   Thread priority (1–99) for real-time policies; ignored under SCHED_OTHER.
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
     * @brief Cancels the scheduler (and any running transmission).
     *
     * Waits for the scheduler thread to stop, and forces any in‐flight
     * transmission to end.
     */

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
     *   Sets the internal stop flag so that ongoing loops in the transmit
     *   thread can exit promptly.
     */
    void requestStopTx();

    /**
     * @brief Stop and wait for the scheduler/transmit threads.
     *
     * @details
     *   Requests stop and joins threads as needed.
     */
    void stopAndJoin();

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
     * settings and symbol sequences are correctly populated before transmission.
     */
    void dumpParameters();

private:
    /**
     * @brief Invoked just before each transmission begins.
     *
     * This callback is fired on the transmit thread immediately before
     * starting either a continuous tone or a WSPR symbol sequence.
     * Users can assign a function via `setTransmissionCallbacks()` to
     * perform any setup or logging when transmission is about to start.
     */
    StartCallback on_transmit_start_{};

    /**
     * @brief Invoked immediately after all WSPR symbols have been sent.
     *
     * This callback is fired on the transmit thread right after the last
     * WSPR symbol is transmitted and before the hardware is torn down.
     * Users can assign a function via `setTransmissionCallbacks()` to
     * perform any cleanup or notification when a WSPR transmission completes.
     */
    EndCallback on_transmit_end_{};

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
     * @brief Condition variable used to wake the transmission thread.
     *
     * requestStopTx() calls notify_all() on this to unblock
     * any waits so the thread can observe stop_requested_.
     */
    std::condition_variable stop_cv_;

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
     * This structure is used to store the mapping between the bus address (used for DMA
     * and peripheral accesses) and the virtual address (used by the application) of a single
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
     * This global variable holds the bus and virtual addresses of the constant memory page,
     * which is used to store fixed data required for DMA operations, such as the tuning words
     * for frequency generation.
     */
    struct PageInfo const_page_;

    /**
     * @brief Page information for the DMA instruction page.
     *
     * This global variable holds the bus and virtual addresses of the DMA instruction page,
     * where DMA control blocks (CBs) are stored. This page is used during the setup and
     * operation of DMA transfers.
     */
    struct PageInfo instr_page_;

    /**
     * @brief Array of page information structures for DMA control blocks.
     *
     * This global array contains the bus and virtual addresses for each page used in the DMA
     * instruction chain. It holds 1024 entries, corresponding to the 1024 DMA control blocks used
     * for managing data transfers.
     */
    struct PageInfo instructions_[1024];

    /**
     * @brief Random frequency offset for standard WSPR transmissions.
     *
     * This constant defines the range, in Hertz, for random frequency offsets
     * applied to standard WSPR transmissions. The offset is applied symmetrically
     * around the target frequency, resulting in a random variation of ±80 Hz.
     *
     * This helps distribute transmissions within the WSPR band, reducing the
     * likelihood of overlapping signals.
     *
     * @note This offset is applicable for standard WSPR transmissions (2-minute cycles).
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

    static constexpr uint32_t GPIO_BUS_BASE = 0x7E200000;
    static constexpr uint32_t CM_GP0CTL_BUS = 0x7E101070;
    static constexpr uint32_t CM_GP0DIV_BUS = 0x7E101074;
    static constexpr uint32_t PADS_GPIO_0_27_BUS = 0x7E10002C;
    static constexpr uint32_t CLK_BUS_BASE = 0x7E101000;
    static constexpr uint32_t DMA_BUS_BASE = 0x7E007000;
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

    static_assert(
        PWM_CLOCKS_PER_ITER_NOMINAL > 0,
        "PWM_CLOCKS_PER_ITER_NOMINAL must be non-zero.");

    static inline constexpr std::array<int, 8> DRIVE_STRENGTH_TABLE = {
        2, 4, 6, 8, 10, 12, 14, 16};

    struct WsprTransmissionParams
    {
        static const std::size_t symbol_count = MSG_SIZE;
        std::array<uint8_t, symbol_count> symbols;

        std::string call_sign;
        std::string grid_square;
        int power_dbm;
        double frequency;
        double ppm;
        bool is_tone;
        int power;
        double symtime;
        double tone_spacing;
        std::vector<double> dma_table_freq;
        bool use_offset;

        WsprTransmissionParams()
            : symbols{},
              frequency(0.0),
              is_tone(false),
              power(0),
              symtime(0.0),
              tone_spacing(0.0),
              dma_table_freq(1024, 0.0),
              use_offset(false)
        {
        }
    };

    struct WsprTransmissionParams trans_params_;

    struct DMAConfig
    {
        double plld_nominal_freq;
        double plld_clock_frequency;
        volatile uint8_t *peripheral_base_virtual;
        uint32_t orig_gp0ctl;
        uint32_t orig_gp0div;
        uint32_t orig_pwm_ctl;
        uint32_t orig_pwm_sta;
        uint32_t orig_pwm_rng1;
        uint32_t orig_pwm_rng2;
        uint32_t orig_pwm_fifocfg;

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

    struct DMAConfig dma_config_;

    struct MailboxStruct
    {
        uint32_t mem_ref = 0;
        std::uintptr_t bus_addr = 0;
        volatile uint8_t *virt_addr = nullptr;
        unsigned pool_size = 0;
        unsigned pool_cnt = 0;
    };

    MailboxStruct mailbox_struct_;

    struct CB
    {
        volatile unsigned int TI;
        volatile unsigned int SOURCE_AD;
        volatile unsigned int DEST_AD;
        volatile unsigned int TXFR_LEN;
        volatile unsigned int STRIDE;
        volatile unsigned int NEXTCONBK;
        volatile unsigned int RES1;
        volatile unsigned int RES2;
    };

    struct GPCTL
    {
        uint32_t SRC : 4;
        uint32_t ENAB : 1;
        uint32_t KILL : 1;
        uint32_t : 1;
        uint32_t BUSY : 1;
        uint32_t FLIP : 1;
        uint32_t MASH : 2;
        uint32_t : 13;
        uint32_t PASSWD : 8;
    };

    static_assert(sizeof(GPCTL) == 4, "GPCTL must be exactly 32 bits.");

    struct DMAregs
    {
        volatile unsigned int CS;
        volatile unsigned int CONBLK_AD;
        volatile unsigned int TI;
        volatile unsigned int SOURCE_AD;
        volatile unsigned int DEST_AD;
        volatile unsigned int TXFR_LEN;
        volatile unsigned int STRIDE;
        volatile unsigned int NEXTCONBK;
        volatile unsigned int DEBUG;
    };

    void fire_start_cb(const std::string &msg, double frequency);
    void fire_end_cb(const std::string &msg, double elapsed);

    void transmit();
    void join_transmission();
    void dma_cleanup();

    constexpr int get_gpio_power_mw(int level);
    inline double convert_mw_dbm(double mw);

    inline volatile int &access_bus_address(std::uintptr_t bus_addr);
    inline void set_bit_bus_address(std::uintptr_t base, unsigned int bit);
    inline void clear_bit_bus_address(std::uintptr_t base, unsigned int bit);

    void get_plld();
    void allocate_memory_pool(unsigned numpages);
    void get_real_mem_page_from_pool(void **vAddr, void **bAddr);
    void deallocate_memory_pool();
    void disable_clock();
    void transmit_on();
    void transmit_off();

    void transmit_symbol(
        const std::uint32_t &sym_num,
        const double &tsym,
        std::uint32_t &bufPtr,
        int symbol_index = -1);

    void clear_dma_setup();
    double bit_trunc(const double &d, const int &lsb);

    void create_dma_pages(
        struct PageInfo &const_page_,
        struct PageInfo &instr_page_,
        struct PageInfo instructions_[]);

    void setup_dma();
    void setup_dma_freq_table(double &center_freq_actual);

    void thread_entry();
    void set_thread_priority();

    class TransmissionScheduler
    {
    public:
        explicit TransmissionScheduler(WsprTransmitter *parent);

        ~TransmissionScheduler();

        void start();

        void stop();

        void notify() noexcept;

    private:
        WsprTransmitter *parent_;
        std::thread thread_;
        std::atomic<bool> stop_requested_{false};
        std::mutex mtx_;
        std::condition_variable cv_;

        std::chrono::system_clock::time_point nextEvent() const;

        void run();
    };

    TransmissionScheduler scheduler_{this};
};

extern WsprTransmitter wsprTransmitter;

#endif // _WSPR_TRANSMIT_HPP
