#include "rp1_gpclk_backend.hpp"

#include <limits>

namespace wsprrypi
{

Rp1GpclkBackend::Rp1GpclkBackend(Rp1GpclkProvider& provider) noexcept
    : provider_(provider) {}

Rp1GpclkBackend::~Rp1GpclkBackend()
{
    if (acquired_ && !in_flight_)
        provider_.release();
}

bool Rp1GpclkBackend::validDrive(std::uint32_t drive_ma) noexcept
{
    return drive_ma == 2 || drive_ma == 4 || drive_ma == 8 || drive_ma == 12;
}

bool Rp1GpclkBackend::prepare(std::uint32_t drive_ma, std::string& error)
{
    if (acquired_)
    {
        error = "RP1 GPCLK backend is already prepared.";
        return false;
    }
    if (!validDrive(drive_ma))
    {
        error = "RP1 GPIO drive must be 2, 4, 8, or 12 mA.";
        return false;
    }
    if (!provider_.acquire(drive_ma, error))
        return false;
    acquired_ = true;
    return true;
}

bool Rp1GpclkBackend::emit(
    const Rp1GpclkPlan& plan, std::size_t tone, std::string& error)
{
    if (!acquired_ || in_flight_ || tone >= plan.tones.size() ||
        plan.fractional_bits != 16 ||
        generation_ == std::numeric_limits<std::uint64_t>::max())
    {
        error = "RP1 GPCLK emit state or plan is invalid.";
        return false;
    }
    const auto& selected = plan.tones[tone];
    if (selected.lower_word_count + selected.upper_word_count != kWritesPerSymbol)
    {
        error = "RP1 GPCLK symbol must contain exactly 66792 divider writes.";
        return false;
    }
    Rp1GpclkProviderProgram program;
    program.lower_divider_word = selected.lower_divider_word;
    program.upper_divider_word = selected.upper_divider_word;
    program.fractional_bits = plan.fractional_bits;
    program.lower_count = selected.lower_word_count;
    program.upper_count = selected.upper_word_count;
    program.writes_per_symbol = kWritesPerSymbol;
    program.tick_divider = kTickDivider;
    program.generation = ++generation_;
    if (!provider_.submit(program, error))
        return false;
    in_flight_ = true;
    return true;
}

bool Rp1GpclkBackend::requestStop(std::string& error)
{
    if (!in_flight_)
        return true;
    return provider_.requestFiniteStop(generation_, error);
}

bool Rp1GpclkBackend::cancel(std::string& error) { return requestStop(error); }
bool Rp1GpclkBackend::timedOut(std::string& error) { return requestStop(error); }

bool Rp1GpclkBackend::cleanup(std::string& error)
{
    if (!acquired_)
        return true;
    if (in_flight_)
    {
        const auto state = provider_.state(generation_);
        if (state != Rp1GpclkCompletionState::complete &&
            state != Rp1GpclkCompletionState::failed)
        {
            error = "RP1 GPCLK descriptor is still draining.";
            return false;
        }
        in_flight_ = false;
    }
    provider_.release();
    acquired_ = false;
    return true;
}

std::uint64_t Rp1GpclkBackend::generation() const noexcept { return generation_; }

} // namespace wsprrypi
