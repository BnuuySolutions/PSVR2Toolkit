#include <SDL3/SDL.h>
#include <SDL3/SDL_loadso.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include "hmd2_gaze.h"
#include "imgui_impl_sdlgpu3.h"
#include <iostream>
#include <string>
#include <vector>
#include <atomic>
#include <cmath>
#include <mutex>
#include <thread>
#include "common.h"

#include "pad_trigger_effect.h"
#include "psvr2tk_capi_loader.h"
#include "psvr2tk_capi_loader_private.h"
#include "gaze_diagnostics.h"
#include "gaze_recorder.h"

std::atomic<bool> g_appRunning = true;
std::mutex g_hapticsMutex;
bool g_playToneLeft = false;
bool g_playToneRight = false;
float g_toneFrequency = 200.0f;
float g_toneAmplitude = 0.5f;

std::mutex g_latestGazeMutex;
hmd2_gaze_status_t g_latestGaze = {};

gaze_capture::GazeRecorder g_gazeRecorder;
gaze_capture::GazeImageDumper g_gazeImageDumper;

std::mutex g_diagnosticsMutex;
gaze_capture::GazeDiagnostics g_gazeDiagnostics;
std::string g_calibrationMessage;

void HapticsThreadFunc() {
  double phase = 0.0;
  unsigned char buf[k_senseChunkSize];

  while (g_appRunning) {
    // Pause thread until it's time to provide the next 32 samples (~93.75Hz)
    psvr2_toolkit_wait_for_pcm();

    bool playL, playR;
    float freq, amp;
    {
      std::scoped_lock<std::mutex> lock(g_hapticsMutex);
      playL = g_playToneLeft;
      playR = g_playToneRight;
      freq = g_toneFrequency;
      amp = g_toneAmplitude;
    }

    // Calculate sine wave phase increment assuming a 3000 Hz sample rate
    double phaseInc = 2.0 * 3.14159265358979323846 * freq / 3000.0;

    for (int i = 0; i < k_senseChunkSize; ++i) {
      buf[i] = static_cast<unsigned char>(static_cast<int8_t>(sin(phase) * 127.0f * amp));
      phase = fmod(phase + phaseInc, 2.0 * 3.14159265358979323846);
    }

    VRControllerType controllerType;
    if (playL && playR) {
      controllerType = VRControllerType::Both;
    } else if (playL) {
      controllerType = VRControllerType::Left;
    } else if (playR) {
      controllerType = VRControllerType::Right;
    } else {
      continue;
    }

    psvr2_toolkit_write_pcm(controllerType, buf);
  }
}

void GazeRecorderThreadFunc() {
  hmd2_gaze_status_t status = {};

  while (g_appRunning) {
    
    if (!psvr2_toolkit_gaze_status(&status, 200)) {
      continue;
    }

    const int64_t hostRecvUs = gaze_capture::NowMicros();

    {
      std::scoped_lock<std::mutex> lock(g_latestGazeMutex);
      g_latestGaze = status;
    }

    {
      std::scoped_lock<std::mutex> lock(g_diagnosticsMutex);
      g_gazeDiagnostics.Update(status, hostRecvUs);
    }

    g_gazeRecorder.Write(status, hostRecvUs);
  }
}

int main(int argc, char *argv[]) {
  // Initialize our CAPI
  SDL_SharedObject *handle = static_cast<SDL_SharedObject *>(psvr2_toolkit_loader_get_module_handle());

  if (!handle) {
    char capiPath[1024] = {};
    psvr2_toolkit_loader_get_module_path(capiPath, sizeof(capiPath));

    std::cerr << "Could not load psvr2_toolkit_capi.dll." << std::endl;
    if (capiPath[0]) {
      std::cerr << "  Tried: " << capiPath << std::endl;
    } else {
      std::cerr << "  The driver has not recorded a path yet. Start SteamVR once with the toolkit installed." << std::endl;
    }

    std::cout << "Press Enter to close" << std::endl;
    std::cin.get();
    return -1;
  }

  // Resolve everything up front and refuse to run on a partial set. Each entry point is an inline shim
  // through a function pointer, so a single unresolved symbol becomes a null call on first use --
  // crashing before a window ever appears, with no module name in the report.
  const int missingPublic = psvr2_toolkit_loader_init_functions(handle);
  const int missingPrivate = psvr2_toolkit_private_loader_init_functions(handle);

  if (missingPublic || missingPrivate) {
    char capiPath[1024] = {};
    psvr2_toolkit_loader_get_module_path(capiPath, sizeof(capiPath));

    std::cerr << "psvr2_toolkit_capi.dll is missing " << (missingPublic + missingPrivate) << " expected function(s)." << std::endl;
    std::cerr << "  Loaded: " << capiPath << std::endl;
    std::cerr << "  This app and that DLL are from different builds. Reinstall the toolkit DLLs from the same" << std::endl;
    std::cerr << "  build as this executable, then try again." << std::endl;

    std::cout << "Press Enter to close" << std::endl;
    std::cin.get();
    return -1;
  }

  int result = psvr2_toolkit_init();
  if (result < 0) {
    std::cerr << "Failed to initialize CAPI! Result: " << result << std::endl;

    std::cout << "Press Enter to close" << std::endl;
    std::cin.get();
    return -1;
  }

  // Initialize SDL
  if (!SDL_Init(SDL_INIT_VIDEO)) {
    std::cerr << "Failed to initialize SDL: " << SDL_GetError() << std::endl;
    return -1;
  }

  // Create Window
  SDL_Window *window = SDL_CreateWindow("PSVR2 CAPI Test", 1280, 720, SDL_WINDOW_RESIZABLE);
  if (!window) {
    std::cerr << "Failed to create window: " << SDL_GetError() << std::endl;
    SDL_Quit();
    return -1;
  }

  // Create GPU Device
  SDL_GPUDevice *gpu_device =
      SDL_CreateGPUDevice(SDL_GPU_SHADERFORMAT_SPIRV | SDL_GPU_SHADERFORMAT_DXIL | SDL_GPU_SHADERFORMAT_MSL | SDL_GPU_SHADERFORMAT_METALLIB, true, nullptr);
  if (!gpu_device) {
    std::cerr << "Failed to create GPU device: " << SDL_GetError() << std::endl;
    SDL_DestroyWindow(window);
    SDL_Quit();
    return -1;
  }

  // Claim window for GPU Device
  if (!SDL_ClaimWindowForGPUDevice(gpu_device, window)) {
    std::cerr << "Failed to claim window for GPU device: " << SDL_GetError() << std::endl;
    SDL_DestroyGPUDevice(gpu_device);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return -1;
  }
  SDL_SetGPUSwapchainParameters(gpu_device, window, SDL_GPU_SWAPCHAINCOMPOSITION_SDR, SDL_GPU_PRESENTMODE_VSYNC);

  // Initialize ImGui
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  ImGui::StyleColorsDark();

  // Setup Platform/Renderer backends
  ImGui_ImplSDL3_InitForSDLGPU(window);
  ImGui_ImplSDLGPU3_InitInfo init_info = {};
  init_info.Device = gpu_device;
  init_info.ColorTargetFormat = SDL_GetGPUSwapchainTextureFormat(gpu_device, window);
  init_info.MSAASamples = SDL_GPU_SAMPLECOUNT_1;
  init_info.SwapchainComposition = SDL_GPU_SWAPCHAINCOMPOSITION_SDR;
  init_info.PresentMode = SDL_GPU_PRESENTMODE_VSYNC;
  ImGui_ImplSDLGPU3_Init(&init_info);

  std::thread hapticsThread(HapticsThreadFunc);
  hapticsThread.detach(); // Detached so if PSVR2TK goes down, the test app can still exit cleanly

  std::thread gazeRecorderThread(GazeRecorderThreadFunc);
  gazeRecorderThread.detach(); // Detached for the same reason as the haptics thread

  // Buffers for CAPI data
  hmd2_gaze_status_t gazeStatus = {};

  std::vector<unsigned char> gazeImage(gaze_capture::GazeImageDumper::k_frameSize, 0);
  uint32_t gazeImageSize = 0;

  // Gaze image is 2048x1024 8-bit grayscale, with a 256-byte header.
  const int IMAGE_WIDTH = 400;
  const int IMAGE_HEIGHT = 200;

  SDL_GPUTextureCreateInfo texture_create_info = {};
  texture_create_info.type = SDL_GPU_TEXTURETYPE_2D;
  texture_create_info.width = IMAGE_WIDTH;
  texture_create_info.height = IMAGE_HEIGHT;
  texture_create_info.layer_count_or_depth = 1;
  texture_create_info.num_levels = 1;
  texture_create_info.format = SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM;
  texture_create_info.usage = SDL_GPU_TEXTUREUSAGE_SAMPLER;
  SDL_GPUTexture *imageTexture = SDL_CreateGPUTexture(gpu_device, &texture_create_info);

  const Uint32 transfer_buffer_size = IMAGE_WIDTH * IMAGE_HEIGHT * 4; // RGBA8
  SDL_GPUTransferBufferCreateInfo transfer_create_info = {};
  transfer_create_info.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
  transfer_create_info.size = transfer_buffer_size;
  SDL_GPUTransferBuffer *transferBuffer = SDL_CreateGPUTransferBuffer(gpu_device, &transfer_create_info);

  bool done = false;
  while (!done) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL3_ProcessEvent(&event);
      if (event.type == SDL_EVENT_QUIT) {
        done = true;
      }
      if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED && event.window.windowID == SDL_GetWindowID(window)) {
        done = true;
      }
    }

    // Fetch the latest data from the CAPI. Gaze status comes from the recorder thread's copy so that
    // there is only ever one caller of psvr2_toolkit_gaze_status; see the note on g_latestGazeMutex.
    {
      std::scoped_lock<std::mutex> lock(g_latestGazeMutex);
      gazeStatus = g_latestGaze;
    }

    uint32_t receivedSize = 0;
    if (psvr2_toolkit_gaze_image_copy(gazeImage.data(), static_cast<uint32_t>(gazeImage.size()), &receivedSize, 0)) {
      gazeImageSize = receivedSize;
      g_gazeImageDumper.MaybeDump(gazeImage.data(), gazeImageSize);
    }

    // Convert and upload texture data
    constexpr uint32_t k_gazeImageHeaderSize = 0x100;
    const uint32_t requiredSize = k_gazeImageHeaderSize + static_cast<uint32_t>(IMAGE_WIDTH) * static_cast<uint32_t>(IMAGE_HEIGHT);
    if (gazeImageSize >= requiredSize) {
      void *mapped_ptr = SDL_MapGPUTransferBuffer(gpu_device, transferBuffer, false);
      if (mapped_ptr) {
        uint32_t *dst = reinterpret_cast<uint32_t *>(mapped_ptr);
        const unsigned char *src = gazeImage.data() + k_gazeImageHeaderSize; // Skip header
        for (size_t i = 0; i < IMAGE_WIDTH * IMAGE_HEIGHT; ++i) {
          unsigned char val = src[i];
          dst[i] = (uint32_t)val | ((uint32_t)val << 8) | ((uint32_t)val << 16) | (0xFF << 24); // Grayscale to RGBA
        }
        SDL_UnmapGPUTransferBuffer(gpu_device, transferBuffer);
      }
    }

    // Start the ImGui frame
    ImGui_ImplSDLGPU3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(1280, 720), ImGuiCond_FirstUseEver);
    ImGui::Begin("PSVR2TK Info");

    ImGui::Text("PSVR2TK Active: %s", psvr2_toolkit_get_driver_active() ? "YES" : "NO");

    if (ImGui::CollapsingHeader("Controller Haptics", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::PushID("HapticsSection");
      std::scoped_lock<std::mutex> lock(g_hapticsMutex);
      ImGui::Checkbox("Play Tone Left", &g_playToneLeft);
      ImGui::Checkbox("Play Tone Right", &g_playToneRight);
      ImGui::SliderFloat("Frequency (Hz)", &g_toneFrequency, 10.0f, 1000.0f);
      ImGui::SliderFloat("Amplitude", &g_toneAmplitude, 0.0f, 1.0f);
      ImGui::PopID();
    }

    if (ImGui::CollapsingHeader("HMD Rumble", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::PushID("HMDRumbleSection");
      static int hmdRumbleHz = 0;
      ImGui::SliderInt("Frequency (Hz)", &hmdRumbleHz, 0, 25);
      if (ImGui::Button("Send HMD Rumble")) {
        psvr2_toolkit_set_hmd_rumble(static_cast<uint8_t>(hmdRumbleHz));
      }
      ImGui::PopID();
    }

    if (ImGui::CollapsingHeader("Trigger Effects", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::PushID("TriggerEffectsSection");
      // Mode dropdown
      static int mode = 0;
      const char *modes[] = {"Off", "Feedback", "Weapon", "Vibration", "MultiplePositionFeedback", "SlopeFeedback", "MultiplePositionVibration"};
      bool modeChanged = ImGui::Combo("Mode", &mode, modes, IM_ARRAYSIZE(modes));
      // Controller type dropdown
      static int controllerType = 0;
      const char *controllerTypes[] = {"Left", "Right", "Both"};
      ImGui::Combo("Controller Type", &controllerType, controllerTypes, IM_ARRAYSIZE(controllerTypes));
      // Parameters based on mode
      static ScePadTriggerEffectCommand payload = {};
      if (modeChanged) {
        payload = {}; // Reset payload when mode changes to avoid garbage values
      }

      payload.mode = static_cast<ScePadTriggerEffectMode>(mode);

      auto SliderUint8 = [](const char *label, uint8_t *v, int v_min, int v_max) {
        uint8_t min = static_cast<uint8_t>(v_min);
        uint8_t max = static_cast<uint8_t>(v_max);
        return ImGui::SliderScalar(label, ImGuiDataType_U8, v, &min, &max, "%u");
      };

      switch (payload.mode) {
      case SCE_PAD_TRIGGER_EFFECT_MODE_OFF:
        break;
      case SCE_PAD_TRIGGER_EFFECT_MODE_FEEDBACK:
        SliderUint8("Position", &payload.commandData.feedbackParam.position, 0, 9);
        SliderUint8("Strength", &payload.commandData.feedbackParam.strength, 0, 8);
        break;
      case SCE_PAD_TRIGGER_EFFECT_MODE_WEAPON:
        SliderUint8("Start Position", &payload.commandData.weaponParam.startPosition, 2, 7);
        SliderUint8("End Position", &payload.commandData.weaponParam.endPosition, payload.commandData.weaponParam.startPosition + 1, 8);
        if (payload.commandData.weaponParam.startPosition < 2) {
          payload.commandData.weaponParam.startPosition = 2;
        }
        if (payload.commandData.weaponParam.startPosition >= payload.commandData.weaponParam.endPosition) {
          payload.commandData.weaponParam.endPosition = payload.commandData.weaponParam.startPosition + 1;
        }
        SliderUint8("Strength", &payload.commandData.weaponParam.strength, 0, 8);
        break;
      case SCE_PAD_TRIGGER_EFFECT_MODE_VIBRATION:
        SliderUint8("Position", &payload.commandData.vibrationParam.position, 0, 9);
        SliderUint8("Amplitude", &payload.commandData.vibrationParam.amplitude, 0, 8);
        SliderUint8("Frequency", &payload.commandData.vibrationParam.frequency, 0, 255);
        break;
      case SCE_PAD_TRIGGER_EFFECT_MODE_MULTIPLE_POSITION_FEEDBACK:
        for (int i = 0; i < SCE_PAD_TRIGGER_EFFECT_CONTROL_POINT_NUM; i++) {
          SliderUint8(("Strength " + std::to_string(i)).c_str(), &payload.commandData.multiplePositionFeedbackParam.strength[i], 0, 8);
        }
        break;
      case SCE_PAD_TRIGGER_EFFECT_MODE_SLOPE_FEEDBACK:
        SliderUint8("Start Position", &payload.commandData.slopeFeedbackParam.startPosition, 0, 8);
        SliderUint8("End Position", &payload.commandData.slopeFeedbackParam.endPosition, payload.commandData.slopeFeedbackParam.startPosition + 1, 9);
        if (payload.commandData.slopeFeedbackParam.startPosition >= payload.commandData.slopeFeedbackParam.endPosition) {
          payload.commandData.slopeFeedbackParam.endPosition = payload.commandData.slopeFeedbackParam.startPosition + 1;
        }
        SliderUint8("Start Strength", &payload.commandData.slopeFeedbackParam.startStrength, 1, 8);
        SliderUint8("End Strength", &payload.commandData.slopeFeedbackParam.endStrength, 1, 8);
        if (payload.commandData.slopeFeedbackParam.startStrength < 1) {
          payload.commandData.slopeFeedbackParam.startStrength = 1;
        }
        if (payload.commandData.slopeFeedbackParam.endStrength < 1) {
          payload.commandData.slopeFeedbackParam.endStrength = 1;
        }
        break;
      case SCE_PAD_TRIGGER_EFFECT_MODE_MULTIPLE_POSITION_VIBRATION:
        SliderUint8("Frequency", &payload.commandData.multiplePositionVibrationParam.frequency, 0, 255);
        for (int i = 0; i < SCE_PAD_TRIGGER_EFFECT_CONTROL_POINT_NUM; i++) {
          SliderUint8(("Amplitude " + std::to_string(i)).c_str(), &payload.commandData.multiplePositionVibrationParam.amplitude[i], 0, 8);
        }
        break;
      }
      if (ImGui::Button("Send Trigger Effect")) {
        psvr2_toolkit_set_trigger_effect(static_cast<VRControllerType>(controllerType), payload);
      }
      ImGui::PopID();
    }

    if (ImGui::CollapsingHeader("Calibration / Gaze Commands", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::PushID("CalibrationSection");

      static GazeCalibrationCommand gazeCmd = {};
      static GazeCalibrationCommand gazeRes = {};
      static bool hasResult = false;

      ImGui::InputScalar("SubCommand", ImGuiDataType_U16, &gazeCmd.status);
      ImGui::InputFloat3("X / Y / Z", &gazeCmd.payload.x);
      ImGui::InputScalar("Result / Enabled Eye", ImGuiDataType_U8, &gazeCmd.payload.result);

      if (ImGui::Button("Send Gaze SET Command")) {
        gazeRes = psvr2_toolkit_private_send_gaze_set_command(gazeCmd);
        hasResult = true;
      }
      ImGui::SameLine();
      if (ImGui::Button("Send Gaze GET Command")) {
        gazeRes = psvr2_toolkit_private_send_gaze_get_command(gazeCmd);
        hasResult = true;
      }

      ImGui::Separator();
      ImGui::Text("Latest Gaze Result:");
      if (hasResult) {
        ImGui::Text("Status: %u", gazeRes.status);
        ImGui::Text("Position: (%.2f, %.2f, %.2f)", gazeRes.payload.x, gazeRes.payload.y, gazeRes.payload.z);
        ImGui::Text("Result Code: %u (%s)", gazeRes.payload.result, gazeRes.payload.result == 0 ? "OK" : "Failed");
      }

      ImGui::PopID();
    }

    if (ImGui::CollapsingHeader("USB Connection", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::PushID("UsbConnectionSection");
      static bool isUsbConnected = true;
      if (ImGui::Checkbox("USB Connected", &isUsbConnected)) {
        psvr2_toolkit_private_set_usb_connection_state(isUsbConnected);
      }
      ImGui::PopID();
    }

    if (ImGui::CollapsingHeader("Gaze Diagnostics", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::PushID("GazeDiagnosticsSection");

      gaze_capture::GazeDiagnosticsSnapshot diag;
      {
        std::scoped_lock<std::mutex> lock(g_diagnosticsMutex);
        diag = g_gazeDiagnostics.Snapshot();
      }

      ImGui::TextWrapped("Derived measurements only -- none of this changes driver behaviour. It exists to "
                         "answer the questions EYE_TRACKING_PLAN.md says must be settled on hardware before "
                         "tasks 4.2, 4.3 and 5.2 can be implemented safely.");
      ImGui::Text("Samples: %llu", static_cast<unsigned long long>(diag.samples));
      ImGui::Separator();

      // measured IPD, for comparison against what SteamVR already reports.
      ImGui::SeparatorText("Measured IPD (task 4.3)");
      if (diag.hasIpd) {
        ImGui::Text("Mean: %.2f mm   (min %.2f, max %.2f, n=%zu)", diag.ipdMeanMm, diag.ipdMinMm, diag.ipdMaxMm, diag.ipdCount);
        ImGui::TextWrapped("Compare against the IPD SteamVR reports. If they disagree, the driver is not "
                           "currently sourcing IPD from the eyes and task 4.3 is worth wiring up.");
      } else {
        ImGui::TextDisabled("No valid pair of gaze origins yet.");
      }
      ImGui::Separator();

      // fit guidance, to calibrate a threshold empirically rather than guessing one.
      ImGui::SeparatorText("Headset fit / pos_guide (task 4.2)");
      if (diag.hasFit) {
        ImGui::Text("Left  mean %.4f  max %.4f", diag.fitLeftMean, diag.fitLeftMax);
        ImGui::Text("Right mean %.4f  max %.4f", diag.fitRightMean, diag.fitRightMax);
        ImGui::Text("Worst right now: %.4f", diag.fitWorstNow);
        ImGui::TextWrapped("Note the value with the headset seated well, then deliberately misfit it and note "
                           "it again. The gap between those two readings is the threshold task 4.2 needs.");
      } else {
        ImGui::TextDisabled("No valid pos_guide samples yet.");
      }
      ImGui::Separator();

      // blink dynamics from the existing binary blink flag.
      ImGui::SeparatorText("Blink dynamics (task 5.2)");
      const gaze_capture::BlinkStats &blinks = diag.blinks;
      ImGui::Text("Blinks: %u   rate %.1f/min", blinks.blinks, blinks.blinksPerMinute);
      ImGui::Text("Duration: last %.1f ms, mean %.1f ms", blinks.lastDurationMs, blinks.meanDurationMs);
      ImGui::Text("Eyes closed: %.2f%% of samples (PERCLOS basis)", blinks.closedFraction * 100.0f);
      ImGui::Separator();

      // convergence distance, which task 2.1 now also uses for the fixation point.
      ImGui::SeparatorText("Convergence distance (tasks 2.1 / 4.4)");
      if (diag.hasConvergence) {
        ImGui::Text("Mean: %.0f mm   (min %.0f, max %.0f)", diag.convMeanMm, diag.convMinMm, diag.convMaxMm);
        ImGui::TextWrapped("This now drives vGazeTarget. If it looks implausible, the driver falls back to "
                           "gazeDefaultFixationDistanceM and the accuracy gain from 2.1 is reduced.");
      } else {
        ImGui::TextDisabled("No valid convergence distance yet.");
      }

      ImGui::Separator();

      // pupillometry.
      ImGui::SeparatorText("Pupillometry (task 5.3)");

      if (!diag.calSolved) {
        ImGui::TextWrapped("Camera axes are NOT calibrated. Foreshortening correction is running with a "
                           "straight-ahead assumption, which on real data leaves most of the gaze-angle "
                           "bias in place. Calibrate below before trusting these numbers.");
      } else {
        ImGui::Text("Camera axes: left az %+.0f el %+.0f, right az %+.0f el %+.0f (CV %.4f)", diag.calLeftAz, diag.calLeftEl, diag.calRightAz, diag.calRightEl,
                    diag.calResidual);
      }

      if (diag.calibrating) {
        ImGui::Text("Collecting... %zu samples. Look slowly around the whole field of view.", diag.calSamples);
        if (ImGui::Button("Solve")) {
          bool solved = false;
          float la = 0.0f, le = 0.0f, ra = 0.0f, re = 0.0f;
          {
            std::scoped_lock<std::mutex> lock(g_diagnosticsMutex);
            solved = g_gazeDiagnostics.SolveCalibration();
            la = g_gazeDiagnostics.CalLeftAz();
            le = g_gazeDiagnostics.CalLeftEl();
            ra = g_gazeDiagnostics.CalRightAz();
            re = g_gazeDiagnostics.CalRightEl();
          }

          if (!solved) {
            g_calibrationMessage = "Not enough 2D gaze spread yet - keep looking around, including up and down.";
          } else {
            // Push the fit into the CAPI so it is persisted beside psvr2_toolkit_capi.dll. Every other
            // client -- the Baballonia module, the Unity calibration app -- reads it from there, so the
            // calibration only has to be done once per headset rather than once per application.
            const int pushed = psvr2_toolkit_pupillometry_set_camera_axes(la, le, ra, re);
            g_calibrationMessage = (pushed == PSVR2TK_RESULT_OK) ? "Solved and saved. Other PSVR2 Toolkit apps will use these axes."
                                                                 : "Solved, but the axes were rejected as implausible and were not saved.";
          }
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
          std::scoped_lock<std::mutex> lock(g_diagnosticsMutex);
          g_gazeDiagnostics.StopCalibration();
        }
      } else {
        if (ImGui::Button("Calibrate camera axes")) {
          std::scoped_lock<std::mutex> lock(g_diagnosticsMutex);
          g_gazeDiagnostics.StartCalibration();
          g_calibrationMessage.clear();
        }
      }

      if (!g_calibrationMessage.empty()) {
        ImGui::TextWrapped("%s", g_calibrationMessage.c_str());
      }

      ImGui::Spacing();

      const PupillometryResult &pupil = diag.pupil;
      if (pupil.valid) {
        ImGui::Text("Corrected: %.3f mm   (%d eye%s)", pupil.correctedMm, pupil.contributingEyes, pupil.contributingEyes == 1 ? "" : "s");
        ImGui::Text("Smoothed:  %.3f mm", pupil.smoothedMm);
        if (pupil.baselineReady) {
          ImGui::Text("Baseline:  %.3f mm    delta %+.3f mm (%+.1f%%)", pupil.baselineMm, pupil.deltaMm, pupil.relative * 100.0f);
        } else {
          ImGui::TextDisabled("Baseline:  building...");
        }
        if (pupil.activityReady) {
          ImGui::Text("Activity:  %.2f dilations/s", pupil.activityIndex);
        } else {
          ImGui::TextDisabled("Activity:  building...");
        }
      } else {
        ImGui::TextDisabled("No accepted pupil sample yet.");
      }

      const uint64_t total = diag.pupilAccepted + diag.pupilRejected;
      ImGui::Text("Accepted %llu / rejected %llu (%.1f%% rejected)", static_cast<unsigned long long>(diag.pupilAccepted),
                  static_cast<unsigned long long>(diag.pupilRejected),
                  total ? 100.0 * static_cast<double>(diag.pupilRejected) / static_cast<double>(total) : 0.0);

      ImGui::TextWrapped("Rejections are mostly blink guard bands, which are intentional -- blink artifacts "
                         "measured 0.27 mm on real data, the same size as the effect being looked for.");
      ImGui::TextWrapped("Prefer 'Activity' over 'delta': there is no display-luminance signal anywhere in the "
                         "headset telemetry, so absolute diameter cannot be separated from the scene simply "
                         "getting brighter. Activity counts abrupt dilations and rides out slow light changes.");

      ImGui::PopID();
    }

    if (ImGui::CollapsingHeader("Gaze Capture", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::PushID("GazeCaptureSection");

      const bool recording = g_gazeRecorder.IsActive();
      if (ImGui::Button(recording ? "Stop Status Recording" : "Start Status Recording")) {
        if (recording) {
          g_gazeRecorder.Stop();
        } else {
          g_gazeRecorder.Start();
        }
      }

      ImGui::Text("Rows: %llu   Frame gaps: %llu", static_cast<unsigned long long>(g_gazeRecorder.Rows()),
                  static_cast<unsigned long long>(g_gazeRecorder.Gaps()));

      const std::string recorderPath = g_gazeRecorder.Path();
      ImGui::TextWrapped("File: %s", recorderPath.empty() ? "(none)" : recorderPath.c_str());

      ImGui::Separator();

      ImGui::SliderInt("Dump every Nth frame", &g_gazeImageDumper.Stride(), 1, 60);
      ImGui::SliderInt("Max frames", &g_gazeImageDumper.MaxFrames(), 1, 2000);

      const bool dumping = g_gazeImageDumper.IsActive();
      if (ImGui::Button(dumping ? "Stop Eye Image Dump" : "Start Eye Image Dump")) {
        if (dumping) {
          g_gazeImageDumper.Stop();
        } else {
          g_gazeImageDumper.Start();
        }
      }

      ImGui::Text("Eye frames written: %d", g_gazeImageDumper.Written());
      ImGui::TextWrapped("Folder: %s", g_gazeImageDumper.Directory().empty() ? "(none)" : g_gazeImageDumper.Directory().c_str());

      ImGui::PopID();
    }

    if (ImGui::CollapsingHeader("Gaze Status", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::Text("Magic: %c%c", gazeStatus.magic[0], gazeStatus.magic[1]);
      ImGui::Text("Version: %u", gazeStatus.version);
      ImGui::Text("Size: %u", gazeStatus.size);

      ImGui::Text("Exp_l: %f", gazeStatus.exp_l);
      ImGui::Text("Exp_r: %f", gazeStatus.exp_r);
      ImGui::Text("Led Status: %u", gazeStatus.led_status);
      ImGui::Text("Exp Counter L: %u", gazeStatus.exp_counter_l);
      ImGui::Text("Exp Counter R: %u", gazeStatus.exp_counter_r);
      ImGui::Text("Led Counter: %u", gazeStatus.led_counter);
      ImGui::Separator();
      ImGui::Text("Wearable Timestamp: %lld", gazeStatus.wearable.timestamp);
      ImGui::Text("Wearable Frame: %u", gazeStatus.wearable.frame_counter);
      ImGui::Text("Left Gaze Origin: (%.2f, %.2f, %.2f)", gazeStatus.wearable.left.gaze_origin_mm.x, gazeStatus.wearable.left.gaze_origin_mm.y,
                  gazeStatus.wearable.left.gaze_origin_mm.z);
      ImGui::Text("Right Gaze Origin: (%.2f, %.2f, %.2f)", gazeStatus.wearable.right.gaze_origin_mm.x, gazeStatus.wearable.right.gaze_origin_mm.y,
                  gazeStatus.wearable.right.gaze_origin_mm.z);
      ImGui::Text("Combined Gaze Dir: (%.2f, %.2f, %.2f)", gazeStatus.wearable.gaze_dir_combined_norm.x, gazeStatus.wearable.gaze_dir_combined_norm.y,
                  gazeStatus.wearable.gaze_dir_combined_norm.z);
      ImGui::Separator();
      ImGui::Text("Foveated Frame: %u", gazeStatus.foveated.frame_counter);
      ImGui::Text("Convergence Distance: %.2f mm", gazeStatus.foveated.convergence_distance_mm);
      ImGui::Text("Foveated Gaze Dir Combined: (%.2f, %.2f, %.2f)", gazeStatus.foveated.gaze_dir_combined_norm.x, gazeStatus.foveated.gaze_dir_combined_norm.y,
                  gazeStatus.foveated.gaze_dir_combined_norm.z);
      ImGui::Separator();
      ImGui::Text("Lens Config Left: (%.2f, %.2f, %.2f)", gazeStatus.lens_config.left.x, gazeStatus.lens_config.left.y, gazeStatus.lens_config.left.z);
      ImGui::Text("Lens Config Right: (%.2f, %.2f, %.2f)", gazeStatus.lens_config.right.x, gazeStatus.lens_config.right.y, gazeStatus.lens_config.right.z);
      ImGui::Text("User Calibration ID: %u", gazeStatus.user_calibration_id);
      ImGui::Text("FR Gaze Origin: (%.2f, %.2f, %.2f)", gazeStatus.fr_gaze_origin.x, gazeStatus.fr_gaze_origin.y, gazeStatus.fr_gaze_origin.z);
      ImGui::Text("Enabled Eye: %u", (uint32_t)gazeStatus.enabled_eye);
      ImGui::Text("Motor Sequence: %u", gazeStatus.motor_sequence);
      ImGui::Text("Motor Strength: %u", gazeStatus.motor_strength);
      ImGui::Text("DSP Return Code: %d", gazeStatus.dsp_return_code);
      ImGui::Separator();
      ImGui::Text("Left Eye Blink: %s", gazeStatus.wearable.left.blink == HMD2_GAZE_BOOL_TRUE ? "Yes" : "No");
      ImGui::Text("Right Eye Blink: %s", gazeStatus.wearable.right.blink == HMD2_GAZE_BOOL_TRUE ? "Yes" : "No");
      ImGui::Text("Pupil Diameter Left: %.2f mm", gazeStatus.wearable.left.pupil_dia_mm);
      ImGui::Text("Pupil Diameter Right: %.2f mm", gazeStatus.wearable.right.pupil_dia_mm);
      ImGui::Text("Gaze Origin Combined Valid: %s", gazeStatus.wearable.is_gaze_origin_combined_valid == HMD2_GAZE_BOOL_TRUE ? "Yes" : "No");
      ImGui::Text("Gaze Dir Combined Valid: %s", gazeStatus.wearable.is_gaze_dir_combined_valid == HMD2_GAZE_BOOL_TRUE ? "Yes" : "No");
    }

    if (ImGui::CollapsingHeader("Gaze Image Stream", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (imageTexture) {
        float aspect = static_cast<float>(IMAGE_WIDTH) / static_cast<float>(IMAGE_HEIGHT);
        float width = ImGui::GetContentRegionAvail().x;
        float height = width / aspect;
        ImGui::Image(imageTexture, ImVec2(width, height));
      }
    }

    ImGui::End();

    // Rendering
    ImGui::Render();
    ImDrawData *draw_data = ImGui::GetDrawData();

    SDL_GPUCommandBuffer *command_buffer = SDL_AcquireGPUCommandBuffer(gpu_device);
    if (command_buffer) {
      SDL_GPUTexture *swapchain_texture;
      SDL_WaitAndAcquireGPUSwapchainTexture(command_buffer, window, &swapchain_texture, nullptr, nullptr);

      if (swapchain_texture) {
        // Upload texture data
        SDL_GPUCopyPass *copy_pass = SDL_BeginGPUCopyPass(command_buffer);
        SDL_GPUTextureTransferInfo source_info = {};
        source_info.transfer_buffer = transferBuffer;
        source_info.offset = 0;

        SDL_GPUTextureRegion dest_region = {};
        dest_region.texture = imageTexture;
        dest_region.w = IMAGE_WIDTH;
        dest_region.h = IMAGE_HEIGHT;
        dest_region.d = 1;
        SDL_UploadToGPUTexture(copy_pass, &source_info, &dest_region, false);
        SDL_EndGPUCopyPass(copy_pass);

        ImGui_ImplSDLGPU3_PrepareDrawData(draw_data, command_buffer);

        SDL_GPUColorTargetInfo target_info = {};
        target_info.texture = swapchain_texture;
        target_info.clear_color = {0.12f, 0.12f, 0.12f, 1.0f};
        target_info.load_op = SDL_GPU_LOADOP_CLEAR;
        target_info.store_op = SDL_GPU_STOREOP_STORE;
        SDL_GPURenderPass *render_pass = SDL_BeginGPURenderPass(command_buffer, &target_info, 1, nullptr);

        ImGui_ImplSDLGPU3_RenderDrawData(draw_data, command_buffer, render_pass);

        SDL_EndGPURenderPass(render_pass);
      }

      SDL_SubmitGPUCommandBuffer(command_buffer);
    }
  }

  // Cleanup
  g_appRunning = false;
  g_gazeRecorder.Stop();

  ImGui_ImplSDLGPU3_Shutdown();
  ImGui_ImplSDL3_Shutdown();
  ImGui::DestroyContext();

  SDL_ReleaseGPUTransferBuffer(gpu_device, transferBuffer);
  SDL_ReleaseGPUTexture(gpu_device, imageTexture);
  SDL_ReleaseWindowFromGPUDevice(gpu_device, window);
  SDL_DestroyGPUDevice(gpu_device);
  SDL_DestroyWindow(window);
  SDL_Quit();

  psvr2_toolkit_deinit();

  return 0;
}