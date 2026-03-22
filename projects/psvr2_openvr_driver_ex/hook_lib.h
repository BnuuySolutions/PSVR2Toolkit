#pragma once

#include "polyhook2/Detour/NatDetour.hpp"

#define INSTALL_STUB(pTarget) psvr2_toolkit::HookLib::InstallStub(pTarget)
#define INSTALL_STUB_ORIGINAL(pTarget, ppOriginal) psvr2_toolkit::HookLib::InstallStub(pTarget, ppOriginal)

#define INSTALL_STUB_RET0(pTarget) psvr2_toolkit::HookLib::InstallStubRet0(pTarget)
#define INSTALL_STUB_RET0_ORIGINAL(pTarget, ppOriginal) psvr2_toolkit::HookLib::InstallStubRet0(pTarget, ppOriginal)

namespace psvr2_toolkit {

  // Provides a thin wrapper around MinHook.
  class HookLib {
  private:
    static void Stub() {}
    static __int64 StubRet0() { return 0; }

  public:
    static void InstallHook(void *pTarget, void *pDetour, void **ppOriginal = nullptr) {
      PLH::NatDetour detour = PLH::NatDetour((uint64_t)pTarget, (uint64_t)pDetour, (uint64_t*)ppOriginal);
      detour.hook();
    }

    static void InstallStub(void *pTarget, void **ppOriginal = nullptr) {
      InstallHook(pTarget, reinterpret_cast<void *>(Stub), ppOriginal);
    }

    static void InstallStubRet0(void *pTarget, void **ppOriginal = nullptr) {
      InstallHook(pTarget, reinterpret_cast<void *>(StubRet0), ppOriginal);
    }

  };

} // psvr2_toolkit
