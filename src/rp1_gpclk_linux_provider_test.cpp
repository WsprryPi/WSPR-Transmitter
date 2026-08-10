#include "rp1_gpclk_linux_provider.hpp"
#include "rp1_gpclk_uapi.h"

#include <cerrno>
#include <cstring>
#include <iostream>
#include <vector>

namespace {
int failures;
void expect(bool c, const char* m) { if (!c) { std::cerr << "FAIL: " << m << '\n'; ++failures; } }

class Io final : public wsprrypi::Rp1GpclkIo {
public:
 int openDevice(const char* p) noexcept override { path=p; return open_result; }
 int control(int, unsigned long r, void* a) noexcept override {
  requests.push_back(r); if (fail_request==r) { error=EINVAL; return -1; }
  if (r==RP1_GPCLK_IOC_ACQUIRE) std::memcpy(&acquire,a,sizeof(acquire));
  if (r==RP1_GPCLK_IOC_SUBMIT) std::memcpy(&program,a,sizeof(program));
  if (r==RP1_GPCLK_IOC_STOP) std::memcpy(&stop,a,sizeof(stop));
  if (r==RP1_GPCLK_IOC_STATE) static_cast<rp1_gpclk_generation*>(a)->state=state_value;
  return 0;
 }
 int closeDevice(int) noexcept override { ++closes; return 0; }
 int lastError() const noexcept override { return error; }
 int open_result{7}, error{ENOENT}, closes{0}; unsigned long fail_request{0};
 unsigned state_value{RP1_GPCLK_STATE_IDLE}; std::string path; std::vector<unsigned long> requests;
 rp1_gpclk_acquire acquire{}; rp1_gpclk_program program{}; rp1_gpclk_generation stop{};
};

void test_wire_contract() {
 Io io; wsprrypi::Rp1GpclkLinuxProvider provider(io); std::string error;
 expect(provider.acquire(2,error), "acquire must succeed");
 expect(io.path=="/dev/rp1-gpclk0" && io.acquire.version==1 && io.acquire.size==sizeof(io.acquire) && io.acquire.drive_ma==2, "acquire must carry version, size, path, and drive");
 wsprrypi::Rp1GpclkProviderProgram p{}; p.fractional_bits=16; p.writes_per_symbol=66792; p.tick_divider=511; p.generation=9; for (std::size_t i=0;i<p.tones.size();++i) p.tones[i]={232445+(i&1),232446,66312,480}; for (std::size_t i=0;i<p.symbols.size();++i) p.symbols[i]=i%4;
 expect(provider.submit(p,error), "submit must succeed");
 expect(io.program.version==1 && io.program.size==sizeof(io.program) && io.program.tones[0].lower_divider_word==232445 && io.program.tones[1].lower_divider_word==232446 && io.program.symbols[161]==1 && io.program.fractional_bits==16, "client must send ordered logical tones and symbol indexes");
 expect(io.program.writes_per_symbol==66792 && io.program.tick_divider==511 && io.program.symbol_count==162 && io.program.tone_count==4 && io.program.generation==9, "client must preserve complete-frame descriptor contract");
 expect(provider.requestFiniteStop(9,error) && io.stop.generation==9, "stop must name generation");
 for (const auto& pair : {std::pair<unsigned,wsprrypi::Rp1GpclkCompletionState>{RP1_GPCLK_STATE_RUNNING,wsprrypi::Rp1GpclkCompletionState::running},{RP1_GPCLK_STATE_DRAINING,wsprrypi::Rp1GpclkCompletionState::draining},{RP1_GPCLK_STATE_COMPLETE,wsprrypi::Rp1GpclkCompletionState::complete},{RP1_GPCLK_STATE_FAILED,wsprrypi::Rp1GpclkCompletionState::failed}}) { io.state_value=pair.first; expect(provider.state(9)==pair.second,"wire state must map to backend state"); }
 provider.release(); expect(io.closes==1 && io.requests.back()==RP1_GPCLK_IOC_RELEASE,"release must issue ioctl before close");
}

void test_failures() {
 Io io; io.open_result=-1; wsprrypi::Rp1GpclkLinuxProvider p(io); std::string e; expect(!p.acquire(2,e) && e.find("No such")!=std::string::npos,"open failure must be reported");
 Io io2; io2.fail_request=RP1_GPCLK_IOC_ACQUIRE; wsprrypi::Rp1GpclkLinuxProvider p2(io2); expect(!p2.acquire(2,e) && io2.closes==1,"acquire ioctl failure must close fd");
}
}
int main() { test_wire_contract(); test_failures(); if(failures) return 1; std::cout << "RP1 GPCLK Linux provider client tests passed\n"; }
