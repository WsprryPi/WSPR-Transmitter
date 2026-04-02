#ifndef WSPR_TRANSMIT_BACKEND_HPP
#define WSPR_TRANSMIT_BACKEND_HPP

#include <cstdint>

#include "wspr_transmit_types.hpp"

class WsprTransmitBackend
{
public:
    virtual ~WsprTransmitBackend() = default;

    virtual void startFaultMonitoring() = 0;
    virtual void stopFaultMonitoring() = 0;
    virtual void prepareTransmission() = 0;
    virtual void configureTransmission(const WsprTransmissionPlan &plan,
                                       double &center_freq_actual) = 0;
    virtual void cleanupTransmission() = 0;
    virtual int getOutputPowerMilliwatts(int level) = 0;
    virtual void beginTransmissionOutput(const WsprTransmissionPlan &plan) = 0;
    virtual void endTransmissionOutput() = 0;
    virtual void emitSymbol(
        const WsprTransmissionPlan &plan,
        const std::uint32_t &sym_num,
        const double &tsym,
        int symbol_index) = 0;
    virtual void resetTransmissionOutput() noexcept = 0;
    virtual bool faulted() const noexcept = 0;
    virtual void clearFault() noexcept = 0;
    virtual void setAutoRecover(bool enable) noexcept = 0;
    virtual bool autoRecoverEnabled() const noexcept = 0;
    virtual bool recoverFromFault() = 0;
    virtual bool recoveryInProgress() const noexcept = 0;
};

#endif
