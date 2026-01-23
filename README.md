<!-- omit in toc -->
# WsprTransmitter

A self-contained C++ class for DMA-driven WSPR (Weak Signal Propagation Reporter)
transmission on Raspberry Pi or other Linux systems. This library can be used
standalone (via the included `main.cpp` demo) or incorporated into other
projects by including `wspr_transmit.hpp` and `wspr_transmit.cpp`.

<!-- omit in toc -->
## Table of Contents

- [Repository Layout](#repository-layout)
- [Dependencies](#dependencies)
- [Building](#building)
- [Public API](#public-api)
  - [Class: `WsprTransmitter`](#class-wsprtransmitter)
    - [Construction & Destruction](#construction--destruction)
    - [Callbacks](#callbacks)
    - [Configuration](#configuration)
    - [Transmission Control](#transmission-control)
    - [Status & Debug](#status--debug)
- [Demo](#demo)
- [License](#license)

---

## Repository Layout

```text
/src
  ├── main.cpp             # Example/demo application
  ├── Makefile             # Build script (assumes dependencies at peer level)
  ├── wspr_transmit.hpp    # Public header
  └── wspr_transmit.cpp    # Implementation

/external
  ├── config_handler.hpp   # (Optional) shared config struct
  ├── config_handler.cpp
  ├── utils.hpp
  └── utils.cpp            # Helper: PPM from chrony
```

## Dependencies

- **WSPR-Message** (symbol generation) — expected at `../../WSPR-Message/src`
- **Broadcom-Mailbox** (DMA/mailbox interface) — expected at
  `../../Broadcom-Mailbox/src`

The current Makefile assumes the dependencies are at the same folder level as
this repo in a larger project.

> **Note:** This is where the `Makefile` includes the dependencies:
>
> ```make
> SUBMODULE_SRCDIRS := $(wildcard ../../WSPR-Message/src)
> SUBMODULE_SRCDIRS += $(wildcard ../../Broadcom-Mailbox/src)
> ```
>
> If your configuration is different, edit `Makefile` accordingly.

---

## Building

To build as a stand-alone demo:

```bash
cd src
make debug
sudo ./build/bin/wspr-transmitter_test
```

- Requires linking against pthreads (`-pthread`).
- Must be run as root (for `/dev/mem` access).

The `Makefile` includes a `help` target:

```bash
$ make help

Available targets:
  release    Build optimized binary
  debug      Build debug binary
  test       Run tests
  gdb        Debug with gdb
  lint       Static analysis
  macros     Show macros
  clean      Remove build artifacts
  help       This message
```

---

## Public API

### Class: `WsprTransmitter`

#### Construction & Destruction

```cpp
WsprTransmitter();        // Default ctor
~WsprTransmitter();       // Stops and cleans up (threads + hardware teardown)
```

#### Callbacks

Install optional callbacks for transmission start/end:

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

- `on_start` fires on the transmit thread immediately before emission begins.
- `on_end` fires on the transmit thread immediately after emission completes.

#### Configuration

Fully initialize frequency, power, PPM, callsign/grid, and optional random
offset:

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

Rebuild the DMA frequency table when PPM changes at runtime:

```cpp
void applyPpmCorrection(double ppm_new);
```

POSIX scheduling for the transmit thread (SCHED_FIFO/RR):

```cpp
void setThreadScheduling(int policy, int priority);
```

#### Transmission Control

```cpp
void startAsync();     // Non-blocking: tone mode or scheduler
void shutdown();       // Stop scheduler + request TX stop + join + hardware off
void requestStopTx();  // Request an in-flight stop (soft stop)
void stopAndJoin();    // Request stop and wait for scheduler/TX threads
```

#### Status & Debug

```cpp
WsprTransmitter::State getState() const noexcept;
void dumpParameters();  // Dumps current config and symbols
```

Example state check:

```cpp
if (wsprTransmitter.getState() == WsprTransmitter::State::TRANSMITTING) {
    // ...
}
```

---

## Demo

The provided `main.cpp` shows a minimal example:

- Pipe-based signal handling to catch `SIGINT`/`SIGTERM`.
- Choose WSPR vs tone mode.
- Configure PPM and other parameters via `configure()`.
- Start transmission with `startAsync()`.
- Wait on condition variable or user input before shutdown.

Build and run:

```bash
cd src
make debug
sudo ./build/bin/wspr-transmitter_test
```

---

## License

MIT — see [LICENSE.md](../LICENSE.md).
