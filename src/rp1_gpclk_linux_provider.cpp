#include "rp1_gpclk_linux_provider.hpp"
#include "rp1_gpclk_uapi.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace wsprrypi
{
int Rp1GpclkPosixIo::openDevice(const char* path) noexcept
{ return ::open(path, O_RDWR | O_CLOEXEC); }
int Rp1GpclkPosixIo::control(int fd, unsigned long request, void* argument) noexcept
{ return ::ioctl(fd, request, argument); }
int Rp1GpclkPosixIo::closeDevice(int fd) noexcept { return ::close(fd); }
int Rp1GpclkPosixIo::lastError() const noexcept { return errno; }

Rp1GpclkLinuxProvider::Rp1GpclkLinuxProvider(
    Rp1GpclkIo& io, std::string device) noexcept
    : io_(io), device_(std::move(device)) {}

Rp1GpclkLinuxProvider::~Rp1GpclkLinuxProvider() { release(); }

bool Rp1GpclkLinuxProvider::failed(const char* operation, std::string& error) const
{
    error = std::string(operation) + ": " + std::strerror(io_.lastError());
    return false;
}

bool Rp1GpclkLinuxProvider::acquire(std::uint32_t drive_ma, std::string& error)
{
    if (fd_ >= 0) { error = "RP1 GPCLK provider is already acquired."; return false; }
    fd_ = io_.openDevice(device_.c_str());
    if (fd_ < 0) return failed("Could not open RP1 GPCLK provider", error);
    rp1_gpclk_acquire request{};
    request.version = RP1_GPCLK_UAPI_VERSION;
    request.size = sizeof(request);
    request.drive_ma = drive_ma;
    if (io_.control(fd_, RP1_GPCLK_IOC_ACQUIRE, &request) < 0) {
        failed("Could not acquire RP1 GPCLK provider", error);
        io_.closeDevice(fd_); fd_ = -1; return false;
    }
    return true;
}

bool Rp1GpclkLinuxProvider::submit(
    const Rp1GpclkProviderProgram& source, std::string& error)
{
    if (fd_ < 0) { error = "RP1 GPCLK provider is not acquired."; return false; }
    rp1_gpclk_program request{};
    request.version = RP1_GPCLK_UAPI_VERSION; request.size = sizeof(request);
    request.fractional_bits = source.fractional_bits;
    request.lower_divider_word = source.lower_divider_word;
    request.upper_divider_word = source.upper_divider_word;
    request.lower_count = source.lower_count; request.upper_count = source.upper_count;
    request.writes_per_symbol = source.writes_per_symbol;
    request.tick_divider = source.tick_divider; request.generation = source.generation;
    if (io_.control(fd_, RP1_GPCLK_IOC_SUBMIT, &request) < 0)
        return failed("Could not submit RP1 GPCLK program", error);
    return true;
}

bool Rp1GpclkLinuxProvider::requestFiniteStop(
    std::uint64_t generation, std::string& error)
{
    rp1_gpclk_generation request{};
    request.version = RP1_GPCLK_UAPI_VERSION; request.size = sizeof(request);
    request.generation = generation;
    if (fd_ < 0 || io_.control(fd_, RP1_GPCLK_IOC_STOP, &request) < 0)
        return failed("Could not request RP1 GPCLK finite stop", error);
    return true;
}

Rp1GpclkCompletionState Rp1GpclkLinuxProvider::state(
    std::uint64_t generation) const noexcept
{
    rp1_gpclk_generation request{};
    request.version = RP1_GPCLK_UAPI_VERSION; request.size = sizeof(request);
    request.generation = generation;
    if (fd_ < 0 || io_.control(fd_, RP1_GPCLK_IOC_STATE, &request) < 0)
        return Rp1GpclkCompletionState::failed;
    switch (request.state) {
    case RP1_GPCLK_STATE_IDLE: return Rp1GpclkCompletionState::idle;
    case RP1_GPCLK_STATE_RUNNING: return Rp1GpclkCompletionState::running;
    case RP1_GPCLK_STATE_DRAINING: return Rp1GpclkCompletionState::draining;
    case RP1_GPCLK_STATE_COMPLETE: return Rp1GpclkCompletionState::complete;
    default: return Rp1GpclkCompletionState::failed;
    }
}

void Rp1GpclkLinuxProvider::release() noexcept
{
    if (fd_ < 0) return;
    io_.control(fd_, RP1_GPCLK_IOC_RELEASE, nullptr);
    io_.closeDevice(fd_); fd_ = -1;
}
} // namespace wsprrypi
