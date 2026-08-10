#include "rp1_gpclk_backend.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace {
int failures = 0;
void expect(bool value, const char* message) { if (!value) { std::cerr << "FAIL: " << message << '\n'; ++failures; } }

class Provider final : public wsprrypi::Rp1GpclkProvider {
public:
 bool acquire(std::uint32_t drive, std::string&) override { drives.push_back(drive); return acquire_ok; }
 bool submit(const wsprrypi::Rp1GpclkProviderProgram& p, std::string&) override { programs.push_back(p); current = wsprrypi::Rp1GpclkCompletionState::running; return submit_ok; }
 bool requestFiniteStop(std::uint64_t g, std::string&) override { stops.push_back(g); current = wsprrypi::Rp1GpclkCompletionState::draining; return true; }
 wsprrypi::Rp1GpclkCompletionState state(std::uint64_t) const noexcept override { return current; }
 void release() noexcept override { ++releases; }
 bool acquire_ok{true}, submit_ok{true}; int releases{0};
 wsprrypi::Rp1GpclkCompletionState current{wsprrypi::Rp1GpclkCompletionState::idle};
 std::vector<std::uint32_t> drives; std::vector<std::uint64_t> stops;
 std::vector<wsprrypi::Rp1GpclkProviderProgram> programs;
};

wsprrypi::Rp1GpclkPlan plan() {
 wsprrypi::Rp1GpclkPlannerInput in; in.center_frequency_hz=14097100; in.tone_spacing_hz=1.46484375; in.parent_frequency_hz=50000000; in.dither_sequence_length=66792;
 return wsprrypi::planRp1GpclkWspr(in).plan;
}

void test_drive_profiles() {
 for (auto drive : {2u,4u,8u,12u}) { Provider p; wsprrypi::Rp1GpclkBackend b(p); std::string e; expect(b.prepare(drive,e), "supported drive must prepare"); expect(b.cleanup(e), "idle backend must clean up"); }
 for (auto drive : {0u,6u,10u,16u}) { Provider p; wsprrypi::Rp1GpclkBackend b(p); std::string e; expect(!b.prepare(drive,e) && p.drives.empty(), "unsupported drive must be rejected before provider"); }
 expect(wsprrypi::Rp1GpclkBackend::kDefaultDriveMa==2, "default drive must be 2 mA");
}

void test_program_and_finite_stop() {
 Provider p; wsprrypi::Rp1GpclkBackend b(p); std::string e; auto planned=plan();
 expect(b.prepare(2,e), "prepare must acquire provider");
 for (std::size_t tone=0; tone<4; ++tone) {
  expect(b.emit(planned,tone,e), "all four profiles must submit"); auto program=p.programs.back();
  expect(program.writes_per_symbol==66792 && program.tick_divider==511, "production timing constants must be preserved");
  expect(program.lower_divider_word==planned.tones[tone].lower_divider_word && program.fractional_bits==16, "provider must receive the unpacked lower divider word");
  expect(program.upper_divider_word==planned.tones[tone].upper_divider_word, "provider must receive the unpacked upper divider word");
  expect(b.cancel(e), "cancel must request finite stop");
  expect(!b.cleanup(e), "cleanup must not release a draining descriptor");
  p.current=wsprrypi::Rp1GpclkCompletionState::complete;
  expect(b.cleanup(e), "cleanup must release after finite completion");
  if (tone != 3) expect(b.prepare(2,e), "channel must be reusable after completion");
 }
 expect(p.releases==4 && p.stops.size()==4, "every generation must stop and release exactly once");
}

void test_timeout_and_generation() {
 Provider p; wsprrypi::Rp1GpclkBackend b(p); std::string e; auto planned=plan(); b.prepare(2,e); b.emit(planned,0,e);
 const auto first=b.generation(); expect(b.timedOut(e), "timeout must use finite-stop path"); expect(p.stops.back()==first, "timeout must identify active generation");
 p.current=wsprrypi::Rp1GpclkCompletionState::complete; b.cleanup(e); b.prepare(2,e); b.emit(planned,1,e);
 expect(b.generation()==first+1, "reuse must advance generation"); p.current=wsprrypi::Rp1GpclkCompletionState::failed; expect(b.cleanup(e), "failed provider generation must still release ownership");
}
}

int main() { test_drive_profiles(); test_program_and_finite_stop(); test_timeout_and_generation(); if (failures) return 1; std::cout << "RP1 GPCLK production backend contract tests passed\n"; }
