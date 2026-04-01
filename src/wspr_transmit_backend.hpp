#ifndef WSPR_TRANSMIT_BACKEND_HPP
#define WSPR_TRANSMIT_BACKEND_HPP

#include <cstdint>

class WsprTransmitBackend
{
public:
    virtual ~WsprTransmitBackend() = default;

    virtual void start_watchdog() = 0;
    virtual void stop_watchdog() = 0;
    virtual void setup_dma() = 0;
    virtual void setup_dma_freq_table(double &center_freq_actual) = 0;
    virtual void dma_cleanup() = 0;
    virtual int get_gpio_power_mw(int level) = 0;
    virtual void transmit_on() = 0;
    virtual void transmit_off() = 0;
    virtual void transmit_symbol(
        const std::uint32_t &sym_num,
        const double &tsym,
        std::uint32_t &bufPtr,
        int symbol_index) = 0;
    virtual void force_dma_reset_sequence() noexcept = 0;
    virtual bool watchdogFaulted() const noexcept = 0;
    virtual void clearWatchdogFault() noexcept = 0;
    virtual void setWatchdogAutoRecover(bool enable) noexcept = 0;
    virtual bool watchdogAutoRecoverEnabled() const noexcept = 0;
    virtual bool recoverFromWatchdogFault() = 0;
    virtual bool recoveryInProgress() const noexcept = 0;
};

#endif
