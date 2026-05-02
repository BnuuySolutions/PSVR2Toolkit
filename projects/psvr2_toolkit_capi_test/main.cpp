#include <SDL3/SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include "imgui_impl_sdlgpu3.h"
#include <iostream>
#include <vector>
#include <atomic>
#include <cmath>
#include <mutex>
#include <thread>

extern "C" {
  __declspec(dllimport) void CAPI_Initialize();
  __declspec(dllimport) void CAPI_GetGazeStatus(unsigned char* pGazeStatus);
  __declspec(dllimport) void CAPI_GetGazeImage(unsigned char* pGazeImage);
  __declspec(dllimport) int CAPI_ClaimPcmSlot();
  __declspec(dllimport) void CAPI_ReleasePcmSlot(int slot);
  __declspec(dllimport) void CAPI_WritePcm(int slot, const unsigned char* pcmLeft, const unsigned char* pcmRight);
  __declspec(dllimport) void CAPI_WaitForPcmUpdate(int slot);
}

std::atomic<bool> g_appRunning = true;
std::mutex g_hapticsMutex;
bool g_playToneLeft = false;
bool g_playToneRight = false;
float g_toneFrequency = 200.0f;
float g_toneAmplitude = 0.5f;

void HapticsThreadFunc() {
    int slot = CAPI_ClaimPcmSlot();
    if (slot < 0) {
        std::cerr << "Failed to claim PCM slot! Are 4 applications already running?" << std::endl;
        return;
    }

    double phaseLeft = 0.0;
    double phaseRight = 0.0;
    unsigned char leftBuf[32];
    unsigned char rightBuf[32];

    while (g_appRunning) {
        // Pause thread until it's time to provide the next 32 samples (~93.75Hz)
        CAPI_WaitForPcmUpdate(slot);

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

        for (int i = 0; i < 32; ++i) {
            if (playL) {
                leftBuf[i] = static_cast<unsigned char>(static_cast<int8_t>(sin(phaseLeft) * 127.0f * amp));
                phaseLeft = fmod(phaseLeft + phaseInc, 2.0 * 3.14159265358979323846);
            } else {
                leftBuf[i] = 0; phaseLeft = 0.0;
            }
            if (playR) {
                rightBuf[i] = static_cast<unsigned char>(static_cast<int8_t>(sin(phaseRight) * 127.0f * amp));
                phaseRight = fmod(phaseRight + phaseInc, 2.0 * 3.14159265358979323846);
            } else {
                rightBuf[i] = 0; phaseRight = 0.0;
            }
        }

        CAPI_WritePcm(slot, leftBuf, rightBuf);
    }
    CAPI_ReleasePcmSlot(slot);
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
  CAPI_Initialize();
  
  std::thread hapticsThread(HapticsThreadFunc);
  hapticsThread.detach(); // Detached so if PSVR2TK goes down, the test app can still exit cleanly

  // Buffers for CAPI data
  std::vector<unsigned char> gazeStatus(0x148, 0);
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
    CAPI_GetGazeStatus(gazeStatus.data());
    CAPI_GetGazeImage(gazeImage.data());

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

    if (ImGui::CollapsingHeader("Haptics", ImGuiTreeNodeFlags_DefaultOpen)) {
      std::scoped_lock<std::mutex> lock(g_hapticsMutex);
      ImGui::Checkbox("Play Tone Left", &g_playToneLeft);
      ImGui::Checkbox("Play Tone Right", &g_playToneRight);
      ImGui::SliderFloat("Frequency (Hz)", &g_toneFrequency, 10.0f, 1000.0f);
      ImGui::SliderFloat("Amplitude", &g_toneAmplitude, 0.0f, 1.0f);
    }

    if (ImGui::CollapsingHeader("Gaze Status Bytes", ImGuiTreeNodeFlags_DefaultOpen)) {
      for (int i = 0; i < 0x148; ++i) {
        ImGui::Text("%02X ", gazeStatus[i]);
        if ((i + 1) % 16 != 0) ImGui::SameLine();
      }
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

  return 0;
}