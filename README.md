<!-- omit in toc -->
# WsprTransmitter

A self-contained C++17 class for **DMA-driven WSPR (Weak Signal Propagation
Reporter)** transmission on Raspberry Pi–class systems. The transmitter uses
the Broadcom mailbox, DMA, and PWM hardware to generate precisely timed RF with
sub-millHertz stability when properly calibrated.

The library can be used:

- **Standalone**, via the included `main.cpp` test/demo program, or
- **Embedded** as a submodule in a larger project by including
  `wspr_transmit.hpp` and `wspr_transmit.cpp`.

This project is designed for **precise timing**, **safe thread control**, and
**runtime reconfiguration**, including the ability to halt an in-progress
transmission, adjust parameters, and restart cleanly.

<!-- omit in toc -->
## Table of Contents

- [Repository Layout](#repository-layout)
- [Dependencies](#dependencies)
- [Building](#building)
- [Runtime Requirements](#runtime-requirements)
- [Public API](#public-api)
  - [Class: `WsprTransmitter`](#class-wsprtransmitter)
    - [Construction & Destruction](#construction--destruction)
    - [Callbacks](#callbacks)
    - [Configuration](#configuration)
    - [Transmission Control](#transmission-control)
    - [Scheduling](#scheduling)
    - [Status & Debug](#status--debug)
- [Demo / Test Rig](#demo--test-rig)
- [Design Notes](#design-notes)
- [License](#license)

---

## Repository Layout

```text
/src
  ├── main.cpp             # Test rig / demo application
  ├── Makefile             # Build rules for standalone use
  ├── wspr_transmit.hpp    # Public API and documentation
  └── wspr_transmit.cpp    # Implementation

/external                  # Optional helpers used by the demo
  ├── config_handler.hpp
  ├── config_handler.cpp
  ├── utils.hpp
  └── utils.cpp            # PPM estimation helper (e.g. from chrony)
```

When used as a submodule, only `wspr_transmit.hpp/.cpp` are required.

---

## Dependencies

This project expects the following sibling projects when built as part of a
larger tree:

- **WSPR-Message**  
  Encodes WSPR symbols and message data.  
  Expected path:

  ```text
  ../../WSPR-Message/src
  ```

- **Mailbox**
  Provides mailbox, DMA-safe memory allocation, and peripheral mapping.  
  Expected path:

  ```text
  ../../Mailbox/src
  ```

The provided `Makefile` automatically includes these paths:

```make
SUBMODULE_SRCDIRS := $(wildcard ../../WSPR-Message/src)
SUBMODULE_SRCDIRS += $(wildcard ../../Broadcom-Mailbox/src)
```

Adjust paths if your project layout differs.

---

## Building

To build the standalone demo:

```bash
cd src
make debug
sudo ./build/bin/wspr-transmitter_test
```

- Requires linking against pthreads (`-pthread`).
- Must be run as root (for `/dev/mem` access).

```text
release    Build optimized binary
debug      Build debug binary
test       Run test target
gdb        Launch under gdb
lint       Static analysis
macros     Dump compile-time macros
clean      Remove build artifacts
help       Show targets
```

---

## Runtime Requirements

- **Root privileges** (`/dev/mem` access required)
- Linux on Raspberry Pi–class hardware
- A free GPIO suitable for GPCLK / PWM output
- CPU isolation and `SCHED_FIFO` recommended for best timing

---

## Public API

### Class: `WsprTransmitter`

#### Construction & Destruction

```cpp
WsprTransmitter();
~WsprTransmitter();
```

The destructor is **safe and blocking**:

- Stops scheduler
- Requests transmit stop
- Joins threads
- Tears down DMA, PWM, and clocks

---

#### Callbacks

Optional hooks for transmission lifecycle:

```cpp
using StartCallback = std::function<void(
    const std::string &msg,
    double frequency)>;

using EndCallback = std::function<void(
    const std::string &msg,
    double elapsed_secs)>;

void setTransmissionCallbacks(
    StartCallback on_start = {},
    EndCallback   on_end   = {});
```

Callbacks execute **on the transmit thread**, not the caller thread.

---

#### Configuration

Configure frequency, power, PPM correction, and optional message content:

```cpp
void configure(
    double frequency,
    int    power,
    double ppm,
    std::string_view call_sign   = {},
    std::string_view grid_square = {},
    int    power_dbm             = 0,
    bool   use_offset            = false);
```

Apply a new PPM correction **after stopping TX**:

```cpp
void applyPpmCorrection(double ppm_new);
```

Configure thread scheduling (recommended for precision):

```cpp
void setThreadScheduling(int policy, int priority);
// Example: SCHED_FIFO, priority 80
```

The scheduler will launch exactly one WSPR transmission and then stop without scheduling further windows.

```cpp
void setOneShot(bool enable) noexcept;
```

WSPR mode bypasses the next-window scheduler and starts immediately (useful for testing):

```cpp
void setTransmitNow(bool enable) noexcept;
```

---

#### Transmission Control

```cpp
void startAsync();    // Non-blocking start
void requestStopTx(); // Stop in-flight transmission and wait
void stopAndJoin();   // Stop scheduler + TX threads
void shutdown();      // Full hardware + thread teardown
void requestSoftOff() noexcept; // Stop after finishing current transmission
void clearSoftOff() noexcept;   // Clears previous soft off
```

**Important guarantee**:

After `requestStopTx()` returns, it is safe to call:

- `configure()`
- `applyPpmCorrection()`
- `startAsync()`

without races or DMA corruption.

---

#### Scheduling

WSPR message mode uses an internal scheduler aligned to WSPR windows:

- Supports one-shot mode
- Supports immediate start (test mode)
- Supports soft-off to prevent new windows

Tone mode bypasses the scheduler and transmits immediately.

---

#### Status & Debug

```cpp
WsprTransmitter::State getState() const noexcept;
void dumpParameters();
static std::string formatFrequencyMHz(double frequency_hz);
```

State values:

- `DISABLED`
- `ENABLED`
- `TRANSMITTING`
- `HUNG` (reserved)

---

## Demo / Test Rig

The included `main.cpp` demonstrates:

- Signal handling (`SIGINT`, `SIGTERM`)
- Safe shutdown paths
- Tone vs WSPR message mode
- Runtime PPM adjustment
- Scheduler and immediate modes

Run it directly to validate hardware, timing, and calibration.

---

## Design Notes

- DMA control blocks are updated in-place for tight symbol timing
- Absolute sleeps use interruptible + spin-tail logic
- Watchdog detects stalled DMA engines
- Thread-safe stop/join prevents reuse hazards
- Designed for global instantiation

---

## License

MIT — see [LICENSE.md](../LICENSE.md).
