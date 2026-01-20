/**
 * @file wspr_transmit.cpp
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

#include "wspr_transmit.hpp" // Class Declarations

#include "wspr_message.hpp" // WSPR Message Submodule
#include "mailbox.hpp"      // Mailbox Submodule
#include "bcm_model.hpp"    // Enumerates processor types

// C++ Standard Library Headers
#include <algorithm> // std::copy_n, std::clamp
#include <cassert>   // assert()
#include <cerrno>
#include <cmath>     // std::round, std::pow, std::floor
#include <cstdint>   // std::uintptr_t
#include <cstring>   // std::memcpy, std::strerror
#include <cstdlib>   // std::rand, RAND_MAX
#include <fstream>   // std::ifstream
#include <iomanip>   // std::setprecision, std::setw, std::setfill
#include <iostream>  // std::cout, std::cerr
#include <optional>  // std::optional
#include <random>    // std::random_device, std::mt19937, std::uniform_real_distribution
#include <sstream>   // std::stringstream
#include <stdexcept> // std::runtime_error
#include <string_view>
#include <system_error>

// POSIX & System-Specific Headers
#include <fcntl.h>    // open flags
#include <sys/mman.h> // mmap, munmap, MAP_SHARED, PROT_READ/WRITE
#include <sys/stat.h> // struct stat, stat()
#include <sys/time.h> // gettimeofday(), struct timeval
#include <unistd.h>   // usleep(), close(), unlink()

#ifdef DEBUG_WSPR_TRANSMIT
constexpr const bool debug = true;
#else
constexpr const bool debug = false;
#endif
inline constexpr std::string_view debug_tag{"[WSPR-Transmitter] "};

// Helper classes and functions in anonymous namespace
namespace
{
    static constexpr size_t NUM_PAGES = 4096;

    class MailboxMemoryPool
    {
        size_t total_size_;
        uint32_t mem_ref_;
        std::uintptr_t bus_addr_;
        volatile uint8_t *virt_addr_;

    public:
        MailboxMemoryPool(unsigned numpages)
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

        volatile uint8_t *virt() const { return virt_addr_; }
        std::uintptr_t bus() const { return bus_addr_; }
    };

    static inline void sleep_until_abs(const struct timespec &ts_target)
    {
        int err;
        while ((err = clock_nanosleep(
                    CLOCK_MONOTONIC,
                    TIMER_ABSTIME,
                    &ts_target,
                    nullptr)) == EINTR)
        {
        }
        if (err != 0)
        {
            throw std::system_error(err,
                                    std::generic_category(),
                                    "WsprTransmitter: clock_nanosleep failed");
        }
    }

    static inline void sleep_until_abs_realtime(const struct timespec &ts_target)
    {
        int err;
        while ((err = clock_nanosleep(
                    CLOCK_REALTIME,
                    TIMER_ABSTIME,
                    &ts_target,
                    nullptr)) == EINTR)
        {
        }
        if (err != 0)
        {
            throw std::system_error(err,
                                    std::generic_category(),
                                    "WsprTransmitter: clock_nanosleep REALTIME failed");
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

    static inline bool gpclk0_wait_not_busy(volatile int &gp0ctl_reg, int max_us)
    {
        const int polls = (max_us <= 0) ? 0 : (max_us / 100);
        for (int i = 0; i < polls; ++i)
        {
            if ((gp0ctl_reg & (1 << 7)) == 0)
                return true;

            struct timespec ts{};
            ts.tv_sec = 0;
            ts.tv_nsec = 100000; // 100us
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
        ctl |= (1u << 5);   // KILL
        ctl |= 0x5A000000u; // PASSWD
        gp0ctl_reg = static_cast<int>(ctl);

        (void)gpclk0_wait_not_busy(gp0ctl_reg, 200000);
    }

} // end anonymous namespace

WsprTransmitter wsprTransmitter;

/* Public Methods */

WsprTransmitter::WsprTransmitter() = default;

WsprTransmitter::~WsprTransmitter()
{
    disableTransmission();
    dma_cleanup();
}

void WsprTransmitter::setTransmissionCallbacks(StartCallback start_cb, EndCallback end_cb)
{
    on_transmit_start_ = std::move(start_cb);
    on_transmit_end_ = std::move(end_cb);
}

void WsprTransmitter::setupTransmission(
    double frequency,
    int power,
    double ppm,
    std::string_view call_sign,
    std::string_view grid_square,
    int power_dbm,
    bool use_offset)
{
    if (dma_setup_done_)
    {
        disableTransmission();
        dma_cleanup();
    }

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
        WsprMessage msg(trans_params_.call_sign, trans_params_.grid_square, trans_params_.power_dbm);
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

    setup_dma();

    dma_config_.plld_clock_frequency =
        dma_config_.plld_nominal_freq * (1 - ppm / 1e6);

    double center_actual = trans_params_.frequency;
    setup_dma_freq_table(center_actual);

    if (trans_params_.frequency != 0.0)
        trans_params_.frequency = center_actual;
}

void WsprTransmitter::updateDMAForPPM(double ppm_new)
{
    dma_config_.plld_clock_frequency =
        dma_config_.plld_nominal_freq * (1.0 - ppm_new / 1e6);

    double center_actual = trans_params_.frequency;
    setup_dma_freq_table(center_actual);
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


void WsprTransmitter::enableTransmission()
{
    stop_requested_.store(false, std::memory_order_release);

    // If the application has requested a soft-off, do not start scheduling.
    if (!trans_params_.is_tone && soft_off_.load(std::memory_order_acquire))
    {
        return;
    }

    const bool immediate = trans_params_.is_tone ||
                           transmit_now_.load(std::memory_order_acquire);

    if (immediate)
    {
        scheduled_start_rt_ns_.store(0, std::memory_order_release);

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

void WsprTransmitter::disableTransmission()
{
    // Set the stop flag first so a newly spawned transmit thread
    // will abort before it touches DMA/PWM state.
    stop_requested_.store(true, std::memory_order_release);
    stop_cv_.notify_all();

    // Stop the scheduler thread and prevent any further launches.
    soft_off_.store(true, std::memory_order_release);
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
}

void WsprTransmitter::stopTransmission()
{
    stop_requested_.store(true);
    stop_cv_.notify_all();
}

void WsprTransmitter::stop()
{
    disableTransmission();
    dma_cleanup();
}

bool WsprTransmitter::isTransmitting() const noexcept
{
    return transmit_on_.load(std::memory_order_acquire);
}

void WsprTransmitter::printParameters()
{
    std::cout << "Call Sign:         "
              << (trans_params_.is_tone ? "N/A" : trans_params_.call_sign) << std::endl;

    std::cout << "Grid Square:       "
              << (trans_params_.is_tone ? "N/A" : trans_params_.grid_square) << std::endl;

    std::cout << "WSPR Frequency:    "
              << std::fixed << std::setprecision(6)
              << (trans_params_.frequency / 1.0e6) << " MHz" << std::endl;

    std::cout << "GPIO Power:        "
              << std::fixed << std::setprecision(1)
              << convert_mw_dbm(get_gpio_power_mw(trans_params_.power)) << " dBm" << std::endl;

    std::cout << "Test Tone:         "
              << (trans_params_.is_tone ? "True" : "False") << std::endl;

    std::cout << "WSPR Symbol Time:  "
              << (trans_params_.is_tone ? "N/A" : std::to_string(trans_params_.symtime) + " s") << std::endl;

    std::cout << "WSPR Tone Spacing: "
              << (trans_params_.is_tone ? "N/A" : std::to_string(trans_params_.tone_spacing) + " Hz") << std::endl;

    std::cout << "DMA Table Size:    "
              << trans_params_.dma_table_freq.size() << std::endl;

    if (trans_params_.is_tone)
    {
        std::cout << "WSPR Symbols:      N/A" << std::endl;
    }
    else
    {
        std::cout << "WSPR Symbols:" << std::endl;
        const int symbol_count = static_cast<int>(trans_params_.symbols.size());
        for (int i = 0; i < symbol_count; ++i)
        {
            std::cout << static_cast<int>(trans_params_.symbols[i]);

            if (i < symbol_count - 1)
            {
                std::cout << ", ";
            }

            if ((i + 1) % 18 == 0 && i < symbol_count - 1)
            {
                std::cout << std::endl;
            }
        }
        std::cout << std::endl;
    }
}

/* Private Methods */

inline void WsprTransmitter::fire_start_cb(const std::string &msg, const double frequency)
{
    if (on_transmit_start_)
    {
        std::thread([cb = on_transmit_start_, msg, frequency]()
                    { cb(msg, frequency); })
            .detach();
    }
}

inline void WsprTransmitter::fire_end_cb(const std::string &msg, double elapsed)
{
    if (on_transmit_end_)
    {
        std::thread([cb = on_transmit_end_, msg, elapsed]()
                    { cb(msg, elapsed); })
            .detach();
    }
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

void WsprTransmitter::transmit()
{
    if (!trans_params_.is_tone && trans_params_.frequency == 0.0)
    {
        fire_end_cb("Skipping transmission (frequency = 0.0).", 0.0);
        return;
    }

    if (shouldStop())
    {
        if (debug)
        {
            std::cerr << debug_tag << "transmit() aborted before start." << std::endl;
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
                self->transmit_off();
            }
        }
    };

    if (trans_params_.is_tone)
    {
        std::uint32_t dummyBuf = 0;

        transmit_on();
        TxOffGuard tx_guard(this);

        while (!shouldStop())
        {
            transmit_symbol(
                0,
                0.0,
                dummyBuf,
                -1);
        }

        transmit_off();
        tx_guard.dismiss();
    }
    else
    {
        std::uint32_t bufPtr = 0;

        // Align to the scheduler-provided realtime boundary before starting TX.
        const std::int64_t start_rt_ns =
            scheduled_start_rt_ns_.load(std::memory_order_acquire);
        if (start_rt_ns != 0)
        {
            struct timespec start_rt{};
            start_rt.tv_sec = start_rt_ns / 1000000000LL;
            start_rt.tv_nsec = static_cast<long>(start_rt_ns % 1000000000LL);

            sleep_until_abs_realtime(start_rt);

            if (debug)
            {
                struct timespec now_rt{};
                clock_gettime(CLOCK_REALTIME, &now_rt);
                std::cerr << debug_tag
                          << "TX start realtime = "
                          << now_rt.tv_sec
                          << "."
                          << std::setw(9) << std::setfill('0') << now_rt.tv_nsec
                          << std::setfill(' ')
                          << std::endl;
            }
        }

        // Fire callback as close to the first symbol as possible.
        fire_start_cb("", trans_params_.frequency);

        // Anchor symbol timing to monotonic clock.
        auto t0_chrono = std::chrono::steady_clock::now();
        struct timespec t0_ts{};
        clock_gettime(CLOCK_MONOTONIC, &t0_ts);

        const int symbol_count = static_cast<int>(trans_params_.symbols.size());
        const double symtime = trans_params_.symtime;

        transmit_on();
        TxOffGuard tx_guard(this);

        for (int i = 0; i < symbol_count && !shouldStop(); ++i)
        {
            const int64_t offset_ns =
                static_cast<int64_t>(std::llround(static_cast<double>(i) * symtime * 1e9));

            timespec target = add_ns(t0_ts, offset_ns);

            sleep_until_abs(target);

            if (debug)
            {
                struct timespec now{};
                clock_gettime(CLOCK_MONOTONIC, &now);

                const int64_t late_ns =
                    (now.tv_sec - target.tv_sec) * 1'000'000'000LL +
                    (now.tv_nsec - target.tv_nsec);

                if (late_ns > 1'000'000) // >1 ms late
                {
                    std::cerr << debug_tag
                              << "Symbol overrun: "
                              << late_ns / 1e6
                              << " ms late"
                              << std::endl;
                }
            }

            if (shouldStop())
            {
                break;
            }

            transmit_symbol(
                static_cast<int>(trans_params_.symbols[i]),
                symtime,
                bufPtr,
                i);
        }

        // Allow the final symbol's queued DMA work to drain before turning off
        // the clock. Without this, we can cut the last symbol short by roughly
        // one symbol period.
        if (!shouldStop())
        {
            const int64_t end_ns =
                static_cast<int64_t>(std::llround(
                    static_cast<double>(symbol_count) * symtime * 1e9));

            const timespec end_target = add_ns(t0_ts, end_ns);
            sleep_until_abs(end_target);
        }

        transmit_off();
        tx_guard.dismiss();

        auto t1 = std::chrono::steady_clock::now();
        double total = std::chrono::duration<double>(t1 - t0_chrono).count();
        total = std::round(total * 1000.0) / 1000.0;

        fire_end_cb("", total);
    }
}

void WsprTransmitter::join_transmission()
{
    if (tx_thread_.joinable())
    {
        tx_thread_.join();
    }
}

void WsprTransmitter::dma_cleanup()
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

    transmit_off();

    access_bus_address(CM_GP0DIV_BUS) = dma_config_.orig_gp0div;
    access_bus_address(CM_GP0CTL_BUS) = dma_config_.orig_gp0ctl;
    access_bus_address(PWM_BUS_BASE + 0x00) = dma_config_.orig_pwm_ctl;
    access_bus_address(PWM_BUS_BASE + 0x04) = dma_config_.orig_pwm_sta;
    access_bus_address(PWM_BUS_BASE + 0x10) = dma_config_.orig_pwm_rng1;
    access_bus_address(PWM_BUS_BASE + 0x20) = dma_config_.orig_pwm_rng2;
    access_bus_address(PWM_BUS_BASE + 0x08) = dma_config_.orig_pwm_fifocfg;

    clear_dma_setup();

    if (dma_config_.peripheral_base_virtual)
    {
        ::mailbox.unMapMem(dma_config_.peripheral_base_virtual, Mailbox::PAGE_SIZE * NUM_PAGES);
        dma_config_.peripheral_base_virtual = nullptr;
    }

    deallocate_memory_pool();

    mailbox.close();

    dma_config_ = DMAConfig();
    mailbox_struct_ = MailboxStruct();
}

constexpr int WsprTransmitter::get_gpio_power_mw(int level)
{
    if (level < 0 || level >= static_cast<int>(DRIVE_STRENGTH_TABLE.size()))
    {
        throw std::out_of_range("WsprTransmitter::get_gpio_power_mw: Drive strength level must be between 0 and 7");
    }
    return DRIVE_STRENGTH_TABLE[level];
}

inline double WsprTransmitter::convert_mw_dbm(double mw)
{
    if (mw <= 0.0)
    {
        throw std::domain_error("WsprTransmitter::convert_mw_dbm: Input power (mW) must be > 0 to compute logarithm");
    }
    return 10.0 * std::log10(mw);
}

void WsprTransmitter::thread_entry()
{
    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    CPU_SET(0, &cpus);
    int aff_ret = pthread_setaffinity_np(pthread_self(),
                                         sizeof(cpus),
                                         &cpus);
    if (aff_ret != 0)
    {
        std::cerr << debug_tag
                  << "thread_entry(): failed to set CPU affinity: "
                  << std::strerror(aff_ret)
                  << std::endl;
    }

    try
    {
        set_thread_priority();
    }
    catch (const std::system_error &e)
    {
        throw std::domain_error(
            std::string("WsprTransmitter::thread_entry(): Error setting thread priority: ") + e.what());
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

inline volatile int &WsprTransmitter::access_bus_address(std::uintptr_t bus_addr)
{
    std::uintptr_t offset = Mailbox::offsetFromBase(bus_addr);
    return *reinterpret_cast<volatile int *>(dma_config_.peripheral_base_virtual + offset);
}

inline void WsprTransmitter::set_bit_bus_address(std::uintptr_t base, unsigned int bit)
{
    access_bus_address(base) |= 1 << bit;
}

inline void WsprTransmitter::clear_bit_bus_address(std::uintptr_t base, unsigned int bit)
{
    access_bus_address(base) &= ~(1 << bit);
}

void WsprTransmitter::get_plld()
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
        base_freq_hz = 500e6;
        break;

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
        std::cerr << "Error: Invalid PLLD frequency; defaulting to 500 MHz" << std::endl;
        dma_config_.plld_nominal_freq = 500e6;
        dma_config_.plld_clock_frequency = 500e6;
    }
}

/**
 * @brief Allocate a pool of DMA‑capable memory pages.
 *
 * @details Uses the Broadcom mailbox interface to:
 *   1. Allocate a contiguous block of physical pages.
 *   2. Lock the block and retrieve its bus address.
 *   3. Map the block into user space for CPU access.
 *   4. Track the pool size and reset the per‑page allocation counter.
 *
 * When called with `numpages = 1025`, one page is reserved for constant data
 * and 1024 pages are used for building the DMA instruction chain.
 *
 * @param numpages Total number of pages to allocate (1 constant + N instruction pages).
 * @throws std::runtime_error if mailbox allocation, locking, or mapping fails.
 */
void WsprTransmitter::allocate_memory_pool(unsigned numpages)
{
    // Allocate a contiguous block of physical pages
    mailbox_struct_.mem_ref = mailbox.memAlloc(
        Mailbox::PAGE_SIZE * numpages,
        Mailbox::BLOCK_SIZE);
    if (mailbox_struct_.mem_ref == 0)
    {
        throw std::runtime_error("Error: memAlloc failed.");
    }

    // Lock the block to obtain its bus address
    mailbox_struct_.bus_addr = mailbox.memLock(mailbox_struct_.mem_ref);
    if (mailbox_struct_.bus_addr == 0)
    {
        mailbox.memFree(mailbox_struct_.mem_ref);
        throw std::runtime_error("Error: memLock failed.");
    }

    // Map the locked pages into user‑space virtual memory
    mailbox_struct_.virt_addr = mailbox.mapMem(mailbox.busToPhysical(mailbox_struct_.bus_addr), Mailbox::PAGE_SIZE * numpages);
    if (mailbox_struct_.virt_addr == nullptr)
    {
        mailbox.memUnlock(mailbox_struct_.mem_ref);
        mailbox.memFree(mailbox_struct_.mem_ref);
        throw std::runtime_error("Error: mapMem failed.");
    }

    // Record pool parameters
    mailbox_struct_.pool_size = numpages; // total pages available
    mailbox_struct_.pool_cnt = 0;         // pages allocated so far
}

/**
 * @brief Retrieves the next available memory page from the allocated pool.
 * @details Provides a virtual and bus address for a memory page in the pool.
 *          If no more pages are available, the function prints an error and exits.
 *
 * @param[out] vAddr Pointer to store the virtual address of the allocated page.
 * @param[out] bAddr Pointer to store the bus address of the allocated page.
 */
void WsprTransmitter::get_real_mem_page_from_pool(void **vAddr, void **bAddr)
{
    // Ensure that we do not exceed the allocated pool size.
    if (mailbox_struct_.pool_cnt >= mailbox_struct_.pool_size)
    {
        throw std::runtime_error("Error: unable to allocate more pages.");
    }

    // Compute the offset for the next available page.
    unsigned offset = mailbox_struct_.pool_cnt * Mailbox::PAGE_SIZE;

    // Retrieve the virtual and bus addresses based on the offset.
    *vAddr = reinterpret_cast<void *>(reinterpret_cast<uintptr_t>(mailbox_struct_.virt_addr) + offset);
    *bAddr = reinterpret_cast<void *>(reinterpret_cast<uintptr_t>(mailbox_struct_.bus_addr) + offset);

    // Increment the count of allocated pages.
    mailbox_struct_.pool_cnt++;
}

/**
 * @brief Deallocates the memory pool.
 * @details Releases the allocated memory by unmapping virtual memory,
 *          unlocking, and freeing the memory via the mailbox interface.
 */
void WsprTransmitter::deallocate_memory_pool()
{
    // Free virtual memory mapping if it was allocated.
    if (mailbox_struct_.virt_addr != nullptr)
    {
        mailbox.unMapMem(mailbox_struct_.virt_addr, mailbox_struct_.pool_size * Mailbox::PAGE_SIZE);
        mailbox_struct_.virt_addr = nullptr; // Prevent dangling pointer usage
    }

    // Free the allocated memory block if it was successfully allocated.
    if (mailbox_struct_.mem_ref != 0)
    {
        mailbox.memUnlock(mailbox_struct_.mem_ref);
        mailbox.memFree(mailbox_struct_.mem_ref);
        mailbox_struct_.mem_ref = 0; // Ensure it does not reference a freed block
    }

    // Reset pool tracking variables
    mailbox_struct_.pool_size = 0;
    mailbox_struct_.pool_cnt = 0;
}

/**
 * @brief Disables the PWM clock.
 * @details Clears the enable bit in the clock control register and waits
 *          until the clock is no longer busy. Ensures proper synchronization.
 */
void WsprTransmitter::disable_clock()
{
    transmit_on_.store(false, std::memory_order_release);

    if (dma_config_.peripheral_base_virtual == nullptr)
        return;

    gpclk0_disable_wait(access_bus_address(CM_GP0CTL_BUS));
}

/**
 * @brief Enables TX by configuring GPIO4 and setting the clock source.
 * @details Configures GPIO4 to use alternate function 0 (GPCLK0), sets the drive
 *          strength, disables any active clock, and then enables the clock with PLLD.
 */
void WsprTransmitter::transmit_on()
{
    // Configure GPIO4 function select (Fsel) to alternate function 0 (GPCLK0).
    // This setting follows Section 6.2 of the ARM Peripherals Manual.
    set_bit_bus_address(GPIO_BUS_BASE, 14);   // Set bit 14
    clear_bit_bus_address(GPIO_BUS_BASE, 13); // Clear bit 13
    clear_bit_bus_address(GPIO_BUS_BASE, 12); // Clear bit 12

    // Set GPIO drive strength, values range from 2mA (-3.4dBm) to 16mA (+10.6dBm)
    access_bus_address(PADS_GPIO_0_27_BUS) = 0x5a000018 + trans_params_.power;

    // Define clock control structure and set PLLD as the clock source.
    struct GPCTL setupword = {6 /*SRC*/, 0, 0, 0, 0, 3, 0x5A};

    // Enable the clock by modifying the control word.
    setupword = {6 /*SRC*/, 1, 0, 0, 0, 3, 0x5A};
    int temp;
    std::memcpy(&temp, &setupword, sizeof(int));

    // Apply clock control settings.
    access_bus_address(CM_GP0CTL_BUS) = temp;

    // Set semaphore
    transmit_on_.store(true);
}

/**
 * @brief Disables the transmitter.
 * @details Turns off the transmission by disabling the clock source.
 */
void WsprTransmitter::transmit_off()
{
    const bool was_on = transmit_on_.load(std::memory_order_acquire);

    // Avoid duplicate "DMA before off" prints during normal end-of-TX
    // shutdown where transmit_off() may be called multiple times.
    if (debug && was_on && dma_config_.peripheral_base_virtual != nullptr)
    {
        const std::uint32_t conblk =
            static_cast<std::uint32_t>(access_bus_address(DMA_BUS_BASE + 0x04));
        const std::uint32_t cs =
            static_cast<std::uint32_t>(access_bus_address(DMA_BUS_BASE + 0x00));

        std::cerr << debug_tag
                  << "DMA before off: CS=0x"
                  << std::hex << cs
                  << " CONBLK_AD=0x" << conblk
                  << std::dec
                  << std::endl;
    }

    // Disable the clock, effectively turning off transmission.
    disable_clock();
}

/**
 * @brief Transmits a symbol for a specified duration using DMA.
 * @details Configures the DMA to transmit a symbol (tone) for the specified
 *          time interval (`tsym`). Uses PWM clocks and adjusts frequency
 *          dynamically based on the desired tone spacing.
 *
 * @param[in] sym_num The symbol number to transmit.
 * @param[in] tsym The duration (seconds) for which the symbol is transmitted.
 * @param[in,out] bufPtr The buffer pointer index for DMA instruction handling.
 */
void WsprTransmitter::transmit_symbol(
    const std::uint32_t &sym_num,
    const double &tsym,
    std::uint32_t &bufPtr,
    int symbol_index)
{
    // Early-exit if a stop was already requested.
    if (shouldStop())
    {
        return;
    }

    constexpr std::uint32_t kMask = 0x3FFu; // 1024 ring
    constexpr std::uint32_t kLead = 64u;    // Keep producer ahead of DMA
    constexpr int kPollSleepUs = 50;        // Short backoff
    constexpr int kMaxWaitUs = 200000;      // 200 ms safety timeout per CB write

    auto dma_conblk_ad = [&]() -> std::uint32_t
    {
        // DMA channel CONBLK_AD register (bus-mapped)
        return static_cast<std::uint32_t>(
            access_bus_address(DMA_BUS_BASE + 0x04));
    };

    auto wait_cb_not_active = [&](std::uint32_t idx) -> bool
    {
        // Wait until DMA is NOT executing this CB.
        const std::uint32_t target =
            static_cast<std::uint32_t>(instructions_[idx].b);

        int waited_us = 0;
        while (!shouldStop())
        {
            const std::uint32_t cur = dma_conblk_ad();
            if (cur != target)
                return true;

            // If DMA isn't advancing, do not deadlock forever.
            if (waited_us >= kMaxWaitUs)
            {
                if (debug)
                {
                    std::cerr << debug_tag
                              << "DMA appears stuck at CONBLK_AD=0x"
                              << std::hex << cur << std::dec
                              << ", forcing stop to avoid deadlock."
                              << std::endl;
                }
                stop_requested_.store(true, std::memory_order_release);
                stop_cv_.notify_all();
                return false;
            }

            usleep(kPollSleepUs);
            waited_us += kPollSleepUs;
        }
        return false;
    };

    auto advance_with_lead = [&]() -> void
    {
        // Keep producer ahead. Only do this when starting a new symbol
        // (bufPtr points at the last written CB).
        bufPtr = (bufPtr + kLead) & kMask;
    };

    const bool is_tone = (tsym == 0.0);
    const int f0_idx = static_cast<int>(sym_num) * 2;
    const int f1_idx = f0_idx + 1;

    // Increase chunk size on 32-bit to reduce CB churn and collision risk.
    std::int64_t nominal = static_cast<std::int64_t>(PWM_CLOCKS_PER_ITER_NOMINAL);
    if (sizeof(void *) == 4)
        nominal *= 16;

    if (nominal < 1)
        nominal = 1;

    // Always push the producer away from DMA head at the start of this call.
    advance_with_lead();

    if (is_tone)
    {
        // Continuous tone.
        while (!shouldStop())
        {
            const std::uint32_t n_pwmclk =
                static_cast<std::uint32_t>(nominal);

            // SOURCE_AD
            bufPtr = (bufPtr + 1) & kMask;
            if (!wait_cb_not_active(bufPtr))
                return;

            reinterpret_cast<CB *>(instructions_[bufPtr].v)->SOURCE_AD =
                static_cast<std::uint32_t>(
                    static_cast<std::uintptr_t>(const_page_.b) +
                    static_cast<std::uintptr_t>(f0_idx * 4));

            // TXFR_LEN
            bufPtr = (bufPtr + 1) & kMask;
            if (!wait_cb_not_active(bufPtr))
                return;

            reinterpret_cast<CB *>(instructions_[bufPtr].v)->TXFR_LEN = n_pwmclk;
        }

        return;
    }

    // WSPR symbol mode.
    //
    // IMPORTANT: TXFR_LEN counts the number of DMA transfers paced by PWM DREQ
    // at the PWM *sample* rate, not at the raw PWM clock rate. The sample rate
    // is PWM clock divided by the PWM range (the DMA table size). On 32-bit
    // builds, using the raw clock here causes each symbol to run ~1024x too
    // long and drift accumulates.
    const std::uint32_t table_size =
        static_cast<std::uint32_t>(
            std::max<std::size_t>(1U, trans_params_.dma_table_freq.size()));

    const double pwm_sample_hz =
        pwm_clock_init_ /
        static_cast<double>(table_size);
    const std::int64_t n_pwmclk_per_sym =
        static_cast<std::int64_t>(std::llround(pwm_sample_hz * tsym));

    if (debug)
    {
        const int total_symbols =
            static_cast<int>(trans_params_.symbols.size());

        std::cerr
            << debug_tag
            << "sym=" << sym_num
            << " idx=";

        if (symbol_index >= 0)
        {
            std::cerr
                << (symbol_index + 1)
                << "/" << total_symbols;
        }
        else
        {
            std::cerr << "-";
        }

        std::cerr
            << " tsym=" << std::setprecision(6) << tsym
            << " pwm_clock_init_=" << std::fixed << std::setprecision(3)
            << pwm_clock_init_ << std::defaultfloat
            << " n_pwmclk_per_sym=" << n_pwmclk_per_sym
            << " PWM_CLOCKS_PER_ITER_NOMINAL=" << PWM_CLOCKS_PER_ITER_NOMINAL
            << std::endl;
    }

    if (n_pwmclk_per_sym <= 0 || n_pwmclk_per_sym > 5'000'000'000LL)
        throw std::runtime_error(
            "transmit_symbol(): invalid n_pwmclk_per_sym (bad PWM clock).");

    std::int64_t n_pwmclk_transmitted = 0;
    std::int64_t n_f0_transmitted = 0;

    // Precompute interpolation ratio outside the loop.
    const double f0_freq = trans_params_.dma_table_freq[f0_idx];
    const double f1_freq = trans_params_.dma_table_freq[f1_idx];
    const double tone_freq =
        trans_params_.frequency - 1.5 * trans_params_.tone_spacing +
        static_cast<double>(sym_num) * trans_params_.tone_spacing;

    const double f0_ratio =
        1.0 - (tone_freq - f0_freq) / (f1_freq - f0_freq);

    while (n_pwmclk_transmitted < n_pwmclk_per_sym &&
           !shouldStop())
    {
        // Compute clocks for this chunk.
        std::int64_t n_pwmclk = nominal;

        // Jitter (retain behavior; keep math in 64-bit).
        n_pwmclk += static_cast<std::int64_t>(std::llround(
            (std::rand() / (RAND_MAX + 1.0) - 0.5) *
            static_cast<double>(n_pwmclk)));

        if (n_pwmclk <= 0)
            n_pwmclk = 1;

        if (n_pwmclk_transmitted + n_pwmclk > n_pwmclk_per_sym)
            n_pwmclk = n_pwmclk_per_sym - n_pwmclk_transmitted;

        // Compute how many of these clocks should be f0.
        std::int64_t n_f0 =
            static_cast<std::int64_t>(std::llround(
                f0_ratio * static_cast<double>(n_pwmclk_transmitted + n_pwmclk))) -
            n_f0_transmitted;

        if (n_f0 < 0)
            n_f0 = 0;
        if (n_f0 > n_pwmclk)
            n_f0 = n_pwmclk;

        const std::int64_t n_f1 = n_pwmclk - n_f0;

        // f0 SOURCE_AD
        bufPtr = (bufPtr + 1) & kMask;
        if (!wait_cb_not_active(bufPtr))
            return;

        reinterpret_cast<CB *>(instructions_[bufPtr].v)->SOURCE_AD =
            static_cast<std::uint32_t>(
                static_cast<std::uintptr_t>(const_page_.b) +
                static_cast<std::uintptr_t>(f0_idx * 4));

        // f0 TXFR_LEN
        bufPtr = (bufPtr + 1) & kMask;
        if (!wait_cb_not_active(bufPtr))
            return;

        reinterpret_cast<CB *>(instructions_[bufPtr].v)->TXFR_LEN =
            static_cast<std::uint32_t>(n_f0);

        // f1 SOURCE_AD
        bufPtr = (bufPtr + 1) & kMask;
        if (!wait_cb_not_active(bufPtr))
            return;

        reinterpret_cast<CB *>(instructions_[bufPtr].v)->SOURCE_AD =
            static_cast<std::uint32_t>(
                static_cast<std::uintptr_t>(const_page_.b) +
                static_cast<std::uintptr_t>(f1_idx * 4));

        // f1 TXFR_LEN
        bufPtr = (bufPtr + 1) & kMask;
        if (!wait_cb_not_active(bufPtr))
            return;

        reinterpret_cast<CB *>(instructions_[bufPtr].v)->TXFR_LEN =
            static_cast<std::uint32_t>(n_f1);

        // Update counters.
        n_pwmclk_transmitted += n_pwmclk;
        n_f0_transmitted += n_f0;
    }
}

/**
 * @brief Disables and resets the DMA engine.
 * @details Ensures that the DMA controller is properly reset before exiting.
 *          If the peripheral memory mapping is not set up, the function returns early.
 */
void WsprTransmitter::clear_dma_setup()
{
    // Turn off transmission.
    transmit_off();

    // Ensure memory-mapped peripherals are initialized before proceeding.
    if (dma_config_.peripheral_base_virtual == nullptr)
    {
        return;
    }

    // Obtain a pointer to the DMA control registers.
    volatile DMAregs *DMA0 = reinterpret_cast<volatile DMAregs *>(&(access_bus_address(DMA_BUS_BASE)));

    // Reset the DMA controller by setting the reset bit (bit 31) in the control/status register.
    DMA0->CS = 1 << 31;
}

/**
 * @brief Truncates a floating-point number at a specified bit position.
 * @details Sets all bits less significant than the given LSB to zero.
 *
 * @param d The input floating-point number to be truncated.
 * @param lsb The least significant bit position to retain.
 * @return The truncated value with lower bits set to zero.
 */
double WsprTransmitter::bit_trunc(const double &d, const int &lsb)
{
    // Compute the truncation factor as a power of 2.
    const double factor = std::pow(2.0, lsb);

    // Truncate the number by dividing, flooring, and multiplying back.
    return std::floor(d / factor) * factor;
}

/**
 * @brief Configures and initializes DMA for PWM signal generation.
 * @details Allocates memory pages, creates DMA control blocks, sets up a
 *          circular inked list of DMA instructions, and configures the
 *          PWM clock and registers.
 *
 * @param[out] const_page_ PageInfo structure for storing constant data.
 * @param[out] instr_page_ PageInfo structure for the initial DMA instruction
 *                       page.
 * @param[out] instructions_ Array of PageInfo structures for DMA instructions.
 */
void WsprTransmitter::create_dma_pages(
    struct PageInfo &const_page_,
    struct PageInfo &instr_page_,
    struct PageInfo instructions_[])
{
    // Allocate memory pool for DMA operation
    allocate_memory_pool(1025);

    // Allocate a memory page for storing constants
    {
        // Allocate a real memory page for constants
        void *tmp_v, *tmp_b;
        get_real_mem_page_from_pool(&tmp_v, &tmp_b);
        const_page_.v = tmp_v;
        const_page_.b = reinterpret_cast<std::uintptr_t>(tmp_b);
    }

    // Initialize instruction counter
    int instrCnt = 0;

    // Allocate memory pages and create DMA instructions
    while (instrCnt < 1024)
    {
        // Allocate a memory page for instructions
        {
            // Allocate a real memory page for this CB page
            void *tmp_v, *tmp_b;
            get_real_mem_page_from_pool(&tmp_v, &tmp_b);
            instr_page_.v = tmp_v;
            instr_page_.b = reinterpret_cast<std::uintptr_t>(tmp_b);
        }

        // Create DMA control blocks (CBs)
        struct CB *instr0 = reinterpret_cast<struct CB *>(instr_page_.v);

        for (int i = 0; i < static_cast<int>(Mailbox::PAGE_SIZE / sizeof(struct CB)); i++)
        {
            // Assign virtual and bus addresses for each instruction
            instructions_[instrCnt].v = static_cast<void *>(static_cast<char *>(instr_page_.v) + sizeof(struct CB) * i);
            instructions_[instrCnt].b = instr_page_.b + static_cast<std::uintptr_t>(sizeof(struct CB) * i);

            // Configure DMA transfer: Source = constant memory page, Destination = PWM FI
            // On 64-bit, const_page_.b is already a uintptr_t; truncate to 32 bits for the register.
            instr0->SOURCE_AD = static_cast<uint32_t>(const_page_.b + 2048);
            instr0->DEST_AD = PWM_BUS_BASE + 0x18; // FIFO1
            instr0->TXFR_LEN = 4;
            instr0->STRIDE = 0;
            instr0->TI = (1 << 6) | (5 << 16) | (1 << 26); // DREQ = 1, PWM = 5, No wide mode
            instr0->RES1 = 0;
            instr0->RES2 = 0;

            // Odd instructions modify the GP0 clock divider instead of PWM FIFO
            if (i % 2)
            {
                instr0->DEST_AD = CM_GP0DIV_BUS;
                instr0->STRIDE = 4;
                instr0->TI = (1 << 26); // No wide mode
            }

            // Link previous instruction to the next in the DMA sequence
            if (instrCnt != 0)
            {
                // On 64-bit, truncate the bus address to 32 bits for the DMA engine:
                reinterpret_cast<volatile CB *>(instructions_[instrCnt - 1].v)
                    ->NEXTCONBK = static_cast<uint32_t>(instructions_[instrCnt].b);
            }

            instr0++;
            instrCnt++;
        }
    }

    // Create a circular linked list of DMA instructions (64-bit safe)
    reinterpret_cast<volatile CB *>(instructions_[1023].v)
        ->NEXTCONBK = static_cast<uint32_t>(instructions_[0].b);
    // Create a circular linked list of DMA instructions (64-bit safe)
    reinterpret_cast<volatile CB *>(instructions_[1023].v)
        ->NEXTCONBK = static_cast<uint32_t>(instructions_[0].b);

    // Configure the PWM clock (disable, set divisor, enable)
    access_bus_address(CLK_BUS_BASE + 40 * 4) = 0x5A000026; // Source = PLLD, disable
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    access_bus_address(CLK_BUS_BASE + 41 * 4) = 0x5A002000; // Set PWM divider to 2 (250MHz)
    access_bus_address(CLK_BUS_BASE + 40 * 4) = 0x5A000016; // Source = PLLD, enable
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // Configure PWM registers
    access_bus_address(PWM_BUS_BASE + 0x0) = 0; // Disable PWM
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    access_bus_address(PWM_BUS_BASE + 0x4) = -1; // Clear status errors
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    access_bus_address(PWM_BUS_BASE + 0x10) = 32; // Set default range
    access_bus_address(PWM_BUS_BASE + 0x20) = 32;
    access_bus_address(PWM_BUS_BASE + 0x0) = -1; // Enable FIFO mode, repeat, serializer, and channel
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    access_bus_address(PWM_BUS_BASE + 0x8) = (1 << 31) | 0x0707; // Enable DMA

    // Obtain the base address as an integer pointer
    //
    // Compute the byte‐offset from the bus base (0x7E000000) to your desired
    // DMA register block (DMA_BUS_BASE).
    std::uintptr_t delta = DMA_BUS_BASE - Mailbox::PERIPH_BUS_BASE;

    // Add the offset, then cast to your register pointer
    volatile uint8_t *dma_base = dma_config_.peripheral_base_virtual + delta;

    // Cast to DMAregs pointer to activate DMA
    volatile struct DMAregs *DMA0 = reinterpret_cast<volatile struct DMAregs *>(dma_base);
    DMA0->CS = 1 << 31; // Reset DMA
    DMA0->CONBLK_AD = 0;
    DMA0->TI = 0;
    // on a 64-bit build, DMA regs are only 32 bits wide
    DMA0->CONBLK_AD = static_cast<uint32_t>(instr_page_.b);
    DMA0->CS = (1 << 0) | (255 << 16); // Enable DMA, priority level 255
}

/**
 * @brief Configure and initialize the DMA system for WSPR transmission.
 *
 * @details Performs the following steps in order:
 *   1. Retrieve and configure the PLLD clock frequency and DMA memory flag.
 *   2. Map the peripheral base address into user space.
 *   3. Save the original clock and PWM register values for later restoration.
 *   4. Open the Broadcom mailbox interface for DMA memory allocation.
 *   5. Allocate and set up DMA control blocks for constants and instruction pages.
 *
 * @throws std::runtime_error if the PLLD clock cannot be determined.
 * @throws std::runtime_error if peripheral memory mapping fails.
 * @throws std::runtime_error if mailbox opening fails.
 */
void WsprTransmitter::setup_dma()
{
    // Open the mailbox
    mailbox.open();

    // PLLD & mem-flag
    get_plld();

    // Map peripherals via mailbox.mapMem()
    uint32_t base = Mailbox::discoverPeripheralBase();
    dma_config_.peripheral_base_virtual = ::mailbox.mapMem(base, Mailbox::PAGE_SIZE * NUM_PAGES /*=0x01000000*/);

    // Snapshot regs
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
            // This may throw on timeout or other failure
            MailboxMemoryPool pool(1025);
            // Success!  Use 'pool' here…
            break;
        }
        catch (const std::system_error &e)
        {
            if (e.code().value() == ETIMEDOUT)
            {
                if (debug)
                    std::cerr << debug_tag << "Timeout (attempt " << attempts << ") allocating memory pool, retrying.";

                // A timeout, let's retry
                if (++attempts >= kMaxAttempts)
                    throw std::runtime_error("Mailbox::setup_dma() Too many mailbox timeouts, giving up");

                // Cleanly close and reopen the mailbox
                try
                {
                    ::mailbox.close();
                }
                catch (...)
                { /* Swallow */
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                ::mailbox.open();

                // And loop to retry
            }
        }
        catch (...)
        {
            // Some other error—propagate
            throw;
        }
    }

    // Build DMA pages
    create_dma_pages(const_page_, instr_page_, instructions_);

    // Done
    dma_setup_done_ = true;

    // Read back the divisor you just wrote
    uint32_t div_reg = static_cast<uint32_t>(
        access_bus_address(CLK_BUS_BASE + 41 * 4));
    uint32_t divisor = (div_reg >> 12) & 0xFFF; // bits 23–12

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

    if (debug)
        std::cerr << debug_tag
                  << "PWM div reg=0x" << std::hex << div_reg << std::dec
                  << " divisor=" << divisor
                  << " pwm_clock_init_=" << std::fixed << std::setprecision(3)
                  << pwm_clock_init_
                  << std::endl;

    if (debug)
        std::cerr
            << debug_tag
            << "Actual PWM clock = "
            << std::fixed << std::setprecision(0)
            << pwm_clock_init_
            << " Hz"
            << std::endl;
}

/**
 * @brief Configures the DMA frequency table for signal generation.
 * @details Generates a tuning word table based on the desired center frequency
 *          and tone spacing, adjusting for hardware limitations if necessary.
 *
 * @param[in] center_freq_desired The desired center frequency in Hz.
 * @param[in] tone_spacing The spacing between frequency tones in Hz.
 * @param[in] plld_actual_freq The actual PLLD clock frequency in Hz.
 * @param[out] center_freq_actual The actual center frequency, which may be adjusted.
 * @param[in,out] const_page_ The PageInfo structure for storing tuning words.
 */
void WsprTransmitter::setup_dma_freq_table(double &center_freq_actual)
{
    // Compute the divider values for the lowest and highest WSPR tones.
    double div_lo = bit_trunc(dma_config_.plld_clock_frequency / (trans_params_.frequency - 1.5 * trans_params_.tone_spacing), -12) + std::pow(2.0, -12);
    double div_hi = bit_trunc(dma_config_.plld_clock_frequency / (trans_params_.frequency + 1.5 * trans_params_.tone_spacing), -12);

    // If the integer portion of dividers differ, adjust the center frequency.
    if (std::floor(div_lo) != std::floor(div_hi))
    {
        center_freq_actual = dma_config_.plld_clock_frequency / std::floor(div_lo) - 1.6 * trans_params_.tone_spacing;
        if (debug && trans_params_.frequency != 0.0)
        {
            std::stringstream temp;
            temp << std::fixed << std::setprecision(6)
                 << debug_tag
                 << "Center frequency has been changed to "
                 << center_freq_actual / 1e6 << " MHz";
            std::cerr << temp.str() << " because of hardware limitations." << std::endl;
        }
    }

    // Initialize tuning word table.
    double tone0_freq = center_freq_actual - 1.5 * trans_params_.tone_spacing;
    std::vector<std::uint32_t> tuning_word(1024);

    // Generate tuning words for WSPR tones.
    for (int i = 0; i < 8; i++)
    {
        double tone_freq = tone0_freq + (i >> 1) * trans_params_.tone_spacing;
        double div = bit_trunc(dma_config_.plld_clock_frequency / tone_freq, -12);

        // Apply rounding for even indices.
        if (i % 2 == 0)
        {
            div += std::pow(2.0, -12);
        }

        tuning_word[i] = static_cast<std::uint32_t>(div * std::pow(2.0, 12));
    }

    // Fill the remaining table with default values.
    for (int i = 8; i < 1024; i++)
    {
        double div = 500 + i;
        tuning_word[i] = static_cast<std::uint32_t>(div * std::pow(2.0, 12));
    }

    // Program the DMA table.
    for (int i = 0; i < 1024; i++)
    {
        trans_params_.dma_table_freq[i] = dma_config_.plld_clock_frequency / (static_cast<double>(tuning_word[i]) / std::pow(2.0, 12));

        // Store values in the memory-mapped page.
        reinterpret_cast<std::uint32_t *>(const_page_.v)[i] = (0x5Au << 24) + tuning_word[i];

        // Ensure adjacent tuning words have the same integer portion for valid tone generation.
        if ((i % 2 == 0) && (i < 8))
        {
            assert((tuning_word[i] & (~0xFFFu)) == (tuning_word[i + 1] & (~0xFFFu)));
        }
    }
}
