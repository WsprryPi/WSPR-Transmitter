#ifndef WSPR_TRANSMIT_BACKEND_RPI_HPP
#define WSPR_TRANSMIT_BACKEND_RPI_HPP

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#include "wspr_transmit_backend.hpp"
#include "wspr_transmit.hpp"

class WsprRpiBackend : public WsprTransmitBackend
{
public:
    explicit WsprRpiBackend(IControllerBridge &owner);
    ~WsprRpiBackend() override;

    void start_watchdog() override;
    void stop_watchdog() override;
    void setup_dma() override;
    void setup_dma_freq_table(double &center_freq_actual) override;
    void dma_cleanup() override;
    int get_gpio_power_mw(int level) override;
    void transmit_on() override;
    void transmit_off() override;
    void transmit_symbol(
        const std::uint32_t &sym_num,
        const double &tsym,
        std::uint32_t &bufPtr,
        int symbol_index) override;
    void force_dma_reset_sequence() noexcept override;
    bool watchdogFaulted() const noexcept override;
    void clearWatchdogFault() noexcept override;
    void setWatchdogAutoRecover(bool enable) noexcept override;
    bool watchdogAutoRecoverEnabled() const noexcept override;
    bool recoverFromWatchdogFault() override;
    bool recoveryInProgress() const noexcept override;

private:
    struct PageInfo
    {
        std::uintptr_t b = 0;
        void *v = nullptr;
    };

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

        DMAConfig();
    };

    struct MailboxStruct
    {
        uint32_t mem_ref = 0;
        std::uintptr_t bus_addr = 0;
        volatile uint8_t *virt_addr = nullptr;
        unsigned pool_size = 0;
        unsigned pool_cnt = 0;
    };

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

    inline volatile int &access_bus_address(std::uintptr_t bus_addr);
    inline void set_bit_bus_address(std::uintptr_t base, unsigned int bit);
    inline void clear_bit_bus_address(std::uintptr_t base, unsigned int bit);
    void get_plld();
    void allocate_memory_pool(unsigned numpages);
    void get_real_mem_page_from_pool(void **vAddr, void **bAddr);
    void deallocate_memory_pool();
    void disable_hardware_sequence();
    void disable_clock();
    double bit_trunc(const double &d, const int &lsb);
    void create_dma_pages(PageInfo &const_page, PageInfo &instr_page, PageInfo instructions[]);
    void request_watchdog_recovery() noexcept;
    void recovery_worker();
    bool recover_from_watchdog_fault_locked();

    IControllerBridge &owner_;

    std::thread watchdog_thread_{};
    std::atomic<bool> watchdog_stop_{true};
    std::atomic<bool> watchdog_faulted_{false};
    std::atomic<bool> watchdog_auto_recover_{true};
    std::atomic<bool> recovery_stop_{false};
    std::atomic<bool> recovery_pending_{false};
    std::atomic<bool> recovery_in_progress_{false};

    mutable std::mutex recovery_rate_mtx_{};
    std::deque<std::chrono::steady_clock::time_point> recovery_attempts_{};
    std::chrono::steady_clock::time_point recovery_defer_until_{};
    int post_recovery_state_{static_cast<int>(WsprTransmitter::State::ENABLED)};

    std::thread recovery_thread_{};
    std::mutex recovery_wait_mtx_;
    std::condition_variable recovery_cv_;
    std::mutex recovery_mtx_;

    std::atomic<std::uint32_t> watchdog_last_conblk_{0};
    std::atomic<std::uint32_t> watchdog_last_txfr_len_{0};
    std::atomic<std::chrono::steady_clock::time_point::rep> watchdog_last_change_ns_{0};

    bool dma_setup_done_{false};
    double pwm_clock_init_{0};
    int watchdog_cpu_{1};
    PageInfo const_page_{};
    PageInfo instr_page_{};
    PageInfo instructions_[1024]{};
    DMAConfig dma_config_{};
    MailboxStruct mailbox_struct_{};

    static constexpr auto kRecoveryWindow = std::chrono::minutes(10);
    static constexpr std::size_t kMaxRecoveriesInWindow = 3;
    static constexpr auto kMinRecoveryInterval = std::chrono::seconds(30);
    static constexpr uint32_t GPIO_BUS_BASE = 0x7E200000;
    static constexpr uint32_t CM_GP0CTL_BUS = 0x7E101070;
    static constexpr uint32_t CM_GP0DIV_BUS = 0x7E101074;
    static constexpr uint32_t PADS_GPIO_0_27_BUS = 0x7E10002C;
    static constexpr uint32_t CLK_BUS_BASE = 0x7E101000;
    static constexpr uint32_t DMA_BUS_BASE = 0x7E007000;
    static constexpr uint32_t PWM_BUS_BASE = 0x7E20C000;

#if INTPTR_MAX == INT32_MAX
    static constexpr std::uint32_t PWM_CLOCKS_PER_ITER_NOMINAL = 50000;
#else
    static constexpr std::uint32_t PWM_CLOCKS_PER_ITER_NOMINAL = 1000;
#endif

    static inline constexpr std::array<int, 8> DRIVE_STRENGTH_TABLE = {
        2, 4, 6, 8, 10, 12, 14, 16};
};

#endif
