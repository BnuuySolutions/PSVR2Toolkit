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

std::atomic<bool> g_appRunning = true;
std::mutex g_hapticsMutex;
bool g_playToneLeft = false;
bool g_playToneRight = false;
float g_toneFrequency = 200.0f;
float g_toneAmplitude = 0.5f;

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
        }
        else if (playL) {
            controllerType = VRControllerType::Left;
        }
        else if (playR) {
            controllerType = VRControllerType::Right;
        }
        else {
            continue;
        }

        psvr2_toolkit_write_pcm(controllerType, buf);
    }
}

int main(int argc, char* argv[]) {
  // Initialize SDL
  if (!SDL_Init(SDL_INIT_VIDEO)) {
    std::cerr << "Failed to initialize SDL: " << SDL_GetError() << std::endl;
    return -1;
  }

  // Create Window
  SDL_Window* window = SDL_CreateWindow("PSVR2 CAPI Test", 1280, 720, SDL_WINDOW_RESIZABLE);
  if (!window) {
    std::cerr << "Failed to create window: " << SDL_GetError() << std::endl;
    SDL_Quit();
    return -1;
  }

  // Create GPU Device
  SDL_GPUDevice* gpu_device = SDL_CreateGPUDevice(SDL_GPU_SHADERFORMAT_SPIRV | SDL_GPU_SHADERFORMAT_DXIL | SDL_GPU_SHADERFORMAT_MSL | SDL_GPU_SHADERFORMAT_METALLIB, true, nullptr);
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
  ImGuiIO& io = ImGui::GetIO(); (void)io;
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

  // Initialize our CAPI
  SDL_SharedObject* handle = static_cast<SDL_SharedObject*>(psvr2_toolkit_loader_get_module_handle());
  psvr2_toolkit_loader_init_functions(handle);
  psvr2_toolkit_private_loader_init_functions(handle);
  
  if (handle && psvr2_toolkit_init() < 0) {
      std::cerr << "Failed to initialize CAPI! Are 4 applications already running?" << std::endl;
      // Continue anyway for the sake of the test app
  }
  
  std::thread hapticsThread(HapticsThreadFunc);
  hapticsThread.detach(); // Detached so if PSVR2TK goes down, the test app can still exit cleanly

  // Buffers for CAPI data
  hmd2_gaze_status_t gazeStatus;
  std::vector<unsigned char> gazeImage(0x200100, 0);

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
  SDL_GPUTexture* imageTexture = SDL_CreateGPUTexture(gpu_device, &texture_create_info);

  const Uint32 transfer_buffer_size = IMAGE_WIDTH * IMAGE_HEIGHT * 4; // RGBA8
  SDL_GPUTransferBufferCreateInfo transfer_create_info = {};
  transfer_create_info.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
  transfer_create_info.size = transfer_buffer_size;
  SDL_GPUTransferBuffer* transferBuffer = SDL_CreateGPUTransferBuffer(gpu_device, &transfer_create_info);

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

    // Fetch the latest data from the CAPI
    psvr2_toolkit_gaze_status(&gazeStatus, 0);
    psvr2_toolkit_gaze_image(gazeImage.data(), 0);

    // Convert and upload texture data
    void* mapped_ptr = SDL_MapGPUTransferBuffer(gpu_device, transferBuffer, false);
    if (mapped_ptr) {
        uint32_t* dst = reinterpret_cast<uint32_t*>(mapped_ptr);
        const unsigned char* src = gazeImage.data() + 0x100; // Skip header
        for (size_t i = 0; i < IMAGE_WIDTH * IMAGE_HEIGHT; ++i) {
            unsigned char val = src[i];
            dst[i] = (uint32_t)val | ((uint32_t)val << 8) | ((uint32_t)val << 16) | (0xFF << 24); // Grayscale to RGBA
        }
        SDL_UnmapGPUTransferBuffer(gpu_device, transferBuffer);
    }

    // Start the ImGui frame
    ImGui_ImplSDLGPU3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(1280, 720), ImGuiCond_FirstUseEver);
    ImGui::Begin("Gaze Info");

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
      ImGui::SliderInt("Frequency (Hz)", &hmdRumbleHz, 0, 255);
      if (ImGui::Button("Send HMD Rumble")) {
        psvr2_toolkit_set_hmd_rumble(static_cast<uint8_t>(hmdRumbleHz));
      }
      ImGui::PopID();
    }

    if (ImGui::CollapsingHeader("Trigger Effects", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::PushID("TriggerEffectsSection");
      // Mode dropdown
      static int mode = 0;
      const char* modes[] = { "Off", "Feedback", "Weapon", "Vibration", "MultiplePositionFeedback", "SlopeFeedback", "MultiplePositionVibration" };
      bool modeChanged = ImGui::Combo("Mode", &mode, modes, IM_ARRAYSIZE(modes));
      // Controller type dropdown
      static int controllerType = 0;
      const char* controllerTypes[] = { "Left", "Right", "Both" };
      ImGui::Combo("Controller Type", &controllerType, controllerTypes, IM_ARRAYSIZE(controllerTypes));
      // Parameters based on mode
      static ScePadTriggerEffectCommand payload = {};
      if (modeChanged) {
          payload = {}; // Reset payload when mode changes to avoid garbage values
      }
      
      payload.mode = static_cast<ScePadTriggerEffectMode>(mode);

      auto SliderUint8 = [](const char* label, uint8_t* v, int v_min, int v_max) {
          uint8_t min = static_cast<uint8_t>(v_min);
          uint8_t max = static_cast<uint8_t>(v_max);
          return ImGui::SliderScalar(label, ImGuiDataType_U8, v, &min, &max, "%u");
      };

      switch (payload.mode) {
        case SCE_PAD_TRIGGER_EFFECT_MODE_OFF:
          break;
        case SCE_PAD_TRIGGER_EFFECT_MODE_FEEDBACK:
          SliderUint8("Position", &payload.commandData.feedbackParam.position, 0, 10);
          SliderUint8("Strength", &payload.commandData.feedbackParam.strength, 0, 10);
          break;
        case SCE_PAD_TRIGGER_EFFECT_MODE_WEAPON:
          SliderUint8("Start Position", &payload.commandData.weaponParam.startPosition, 0, 10);
          SliderUint8("End Position", &payload.commandData.weaponParam.endPosition, 0, 10);
          SliderUint8("Strength", &payload.commandData.weaponParam.strength, 0, 10);
          break;
        case SCE_PAD_TRIGGER_EFFECT_MODE_VIBRATION:
          SliderUint8("Position", &payload.commandData.vibrationParam.position, 0, 10);
          SliderUint8("Amplitude", &payload.commandData.vibrationParam.amplitude, 0, 10);
          SliderUint8("Frequency", &payload.commandData.vibrationParam.frequency, 0, 255);
          break;
        case SCE_PAD_TRIGGER_EFFECT_MODE_MULTIPLE_POSITION_FEEDBACK:
          for (int i = 0; i < 10; i++) {
            SliderUint8(("Strength " + std::to_string(i)).c_str(), &payload.commandData.multiplePositionFeedbackParam.strength[i], 0, 10);
          }
          break;
        case SCE_PAD_TRIGGER_EFFECT_MODE_SLOPE_FEEDBACK:
          SliderUint8("Start Position", &payload.commandData.slopeFeedbackParam.startPosition, 0, 10);
          SliderUint8("End Position", &payload.commandData.slopeFeedbackParam.endPosition, 0, 10);
          SliderUint8("Start Strength", &payload.commandData.slopeFeedbackParam.startStrength, 0, 10);
          SliderUint8("End Strength", &payload.commandData.slopeFeedbackParam.endStrength, 0, 10);
          break;
        case SCE_PAD_TRIGGER_EFFECT_MODE_MULTIPLE_POSITION_VIBRATION:
          SliderUint8("Frequency", &payload.commandData.multiplePositionVibrationParam.frequency, 0, 255);
          for (int i = 0; i < 10; i++) {
            SliderUint8(("Amplitude " + std::to_string(i)).c_str(), &payload.commandData.multiplePositionVibrationParam.amplitude[i], 0, 10);
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
      
      static GazeCalibrationCommand gazeCmd = { };
      static GazeCalibrationCommand gazeRes = { };
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
      ImGui::Text("Left Gaze Origin: (%.2f, %.2f, %.2f)", gazeStatus.wearable.left.gaze_origin_mm.x, gazeStatus.wearable.left.gaze_origin_mm.y, gazeStatus.wearable.left.gaze_origin_mm.z);
      ImGui::Text("Right Gaze Origin: (%.2f, %.2f, %.2f)", gazeStatus.wearable.right.gaze_origin_mm.x, gazeStatus.wearable.right.gaze_origin_mm.y, gazeStatus.wearable.right.gaze_origin_mm.z);
      ImGui::Text("Combined Gaze Dir: (%.2f, %.2f, %.2f)", gazeStatus.wearable.gaze_dir_combined_norm.x, gazeStatus.wearable.gaze_dir_combined_norm.y, gazeStatus.wearable.gaze_dir_combined_norm.z);
      ImGui::Separator();
      ImGui::Text("Foveated Frame: %u", gazeStatus.foveated.frame_counter);
      ImGui::Text("Convergence Distance: %.2f mm", gazeStatus.foveated.convergence_distance_mm);
      ImGui::Text("Foveated Gaze Dir Combined: (%.2f, %.2f, %.2f)", gazeStatus.foveated.gaze_dir_combined_norm.x, gazeStatus.foveated.gaze_dir_combined_norm.y, gazeStatus.foveated.gaze_dir_combined_norm.z);
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
    ImDrawData* draw_data = ImGui::GetDrawData();

    SDL_GPUCommandBuffer* command_buffer = SDL_AcquireGPUCommandBuffer(gpu_device);
    if (command_buffer) {
        SDL_GPUTexture* swapchain_texture;
        SDL_WaitAndAcquireGPUSwapchainTexture(command_buffer, window, &swapchain_texture, nullptr, nullptr);

        if (swapchain_texture) {
            // Upload texture data
            SDL_GPUCopyPass* copy_pass = SDL_BeginGPUCopyPass(command_buffer);
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
            target_info.clear_color = { 0.12f, 0.12f, 0.12f, 1.0f };
            target_info.load_op = SDL_GPU_LOADOP_CLEAR;
            target_info.store_op = SDL_GPU_STOREOP_STORE;
            SDL_GPURenderPass* render_pass = SDL_BeginGPURenderPass(command_buffer, &target_info, 1, nullptr);

            ImGui_ImplSDLGPU3_RenderDrawData(draw_data, command_buffer, render_pass);

            SDL_EndGPURenderPass(render_pass);
        }

        SDL_SubmitGPUCommandBuffer(command_buffer);
    }
  }

  // Cleanup
  g_appRunning = false;

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