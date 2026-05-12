# Required for the DLL (version.dll) hijack.
enable_language(ASM_MASM)

add_library(psvr2_toolkit_redir SHARED
  dllmain.c
  version.asm
  version.def
)
set_target_properties(psvr2_toolkit_redir PROPERTIES OUTPUT_NAME version)
