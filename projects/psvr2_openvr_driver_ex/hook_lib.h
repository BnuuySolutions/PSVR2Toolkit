#pragma once

#include <minhook.h>

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
    static bool Initialize() {
      return MH_Initialize() == MH_OK;
    }

    static void InstallHook(void *pTarget, void* pDetour, void **ppOriginal = nullptr) {
      MH_CreateHook(pTarget, pDetour, ppOriginal);
      MH_EnableHook(pTarget);
    }

    static void InstallStub(void* pTarget, void** ppOriginal = nullptr) {
      InstallHook(pTarget, reinterpret_cast<void *>(Stub), ppOriginal);
    }

    static void InstallStubRet0(void *pTarget, void **ppOriginal = nullptr) {
      InstallHook(pTarget, reinterpret_cast<void *>(StubRet0), ppOriginal);
    }

  };

} // psvr2_toolkit
