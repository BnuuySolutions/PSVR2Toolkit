#pragma once

#include "polyhook2/Detour/NatDetour.hpp"

#include "util.h"

#define INSTALL_STUB(pTarget) psvr2_toolkit::HookLib::InstallStub(pTarget)
#define INSTALL_STUB_ORIGINAL(pTarget, ppOriginal) psvr2_toolkit::HookLib::InstallStub(pTarget, ppOriginal)

#define INSTALL_STUB_RET0(pTarget) psvr2_toolkit::HookLib::InstallStubRet0(pTarget)
#define INSTALL_STUB_RET0_ORIGINAL(pTarget, ppOriginal) psvr2_toolkit::HookLib::InstallStubRet0(pTarget, ppOriginal)

namespace psvr2_toolkit {

// Provides a thin wrapper around PolyHook 2.0.
class HookLib {
private:
  static void Stub() {}
  static __int64 StubRet0() { return 0; }

public:
  // Returns false if the detour could not be installed. A failed hook is otherwise indistinguishable
  // from a hook whose target simply never runs, so the log line here is the only signal callers get.
  static bool InstallHook(void *pTarget, void *pDetour, void **ppOriginal = nullptr) {
    // The detour outlives this call and writes the trampoline through this pointer, so it must not
    // be a local. Only used if ppOriginal is null.
    static uint64_t s_discardedOriginal = 0;

    const uint64_t targetAddress = reinterpret_cast<uint64_t>(pTarget);
    PLH::NatDetour *detour = new PLH::NatDetour(targetAddress, (uint64_t)pDetour, ppOriginal ? (uint64_t *)ppOriginal : &s_discardedOriginal);

    if (!detour->hook()) {
      Util::DriverLog("[HookLib] Failed to install hook at {:#x}", targetAddress);
      // Deliberately not deleted. A failed hook may have already patched bytes, and letting the
      // destructor decide what to unwind is riskier than leaking one small object during startup --
      // which is what this code did for every detour, successful or not, before logging was added.
      return false;
    }

    return true;
  }

  static bool InstallStub(void *pTarget, void **ppOriginal = nullptr) { return InstallHook(pTarget, reinterpret_cast<void *>(Stub), ppOriginal); }

  static bool InstallStubRet0(void *pTarget, void **ppOriginal = nullptr) { return InstallHook(pTarget, reinterpret_cast<void *>(StubRet0), ppOriginal); }

  static bool SetInstructionNOPAtAddress(void *pTarget, size_t length) {
    DWORD oldProtect;
    if (!VirtualProtect(pTarget, length, PAGE_EXECUTE_READWRITE, &oldProtect)) {
      return false;
    }
    memset(pTarget, 0x90, length);
    VirtualProtect(pTarget, length, oldProtect, &oldProtect);
    return true;
  }
};

} // namespace psvr2_toolkit
