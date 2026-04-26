#ifdef _WIN32
#include <windows.h>
#endif // defined(_WIN32)
#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_sdlgpu3.h"
#include <stdio.h>          // printf, fprintf

#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

struct AppState {
  SDL_Window* window;
  SDL_GPUDevice* gpu_device;
};

SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[]) {
  AppState* app_state = new AppState;
  *appstate = app_state;
  if (!SDL_Init(SDL_INIT_VIDEO))
    {
      printf("Error: SDL_Init(): %s\n", SDL_GetError());
      return SDL_APP_FAILURE;
    }

         // Create SDL window graphics context
  float main_scale = SDL_GetDisplayContentScale(SDL_GetPrimaryDisplay());
  SDL_WindowFlags window_flags = SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIDDEN | SDL_WINDOW_HIGH_PIXEL_DENSITY;
  app_state->window = SDL_CreateWindow("Dear ImGui SDL3+SDL_GPU example", (int)(1280 * main_scale), (int)(800 * main_scale), window_flags);
  if (app_state->window == nullptr)
    {
      printf("Error: SDL_CreateWindow(): %s\n", SDL_GetError());
      return SDL_APP_FAILURE;
    }
  SDL_SetWindowPosition(app_state->window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
  SDL_ShowWindow(app_state->window);

         // Create GPU Device
  app_state->gpu_device = SDL_CreateGPUDevice(SDL_GPU_SHADERFORMAT_SPIRV | SDL_GPU_SHADERFORMAT_DXIL | SDL_GPU_SHADERFORMAT_MSL | SDL_GPU_SHADERFORMAT_METALLIB, true, nullptr);
  if (app_state->gpu_device == nullptr)
    {
      printf("Error: SDL_CreateGPUDevice(): %s\n", SDL_GetError());
      return SDL_APP_FAILURE;
    }

         // Claim window for GPU Device
  if (!SDL_ClaimWindowForGPUDevice(app_state->gpu_device, app_state->window))
    {
      printf("Error: SDL_ClaimWindowForGPUDevice(): %s\n", SDL_GetError());
      return SDL_APP_FAILURE;
    }
  SDL_SetGPUSwapchainParameters(app_state->gpu_device, app_state->window, SDL_GPU_SWAPCHAINCOMPOSITION_SDR, SDL_GPU_PRESENTMODE_VSYNC);

         // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO(); (void)io;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls

         // Setup Dear ImGui style
  ImGui::StyleColorsDark();
  //ImGui::StyleColorsLight();

         // Setup scaling
  ImGuiStyle& style = ImGui::GetStyle();
  style.ScaleAllSizes(main_scale);        // Bake a fixed style scale. (until we have a solution for dynamic style scaling, changing this requires resetting Style + calling this again)
  style.FontScaleDpi = main_scale;        // Set initial font scale. (in docking branch: using io.ConfigDpiScaleFonts=true automatically overrides this for every window depending on the current monitor)

         // Setup Platform/Renderer backends
  ImGui_ImplSDL3_InitForSDLGPU(app_state->window);
  ImGui_ImplSDLGPU3_InitInfo init_info = {};
  init_info.Device = app_state->gpu_device;
  init_info.ColorTargetFormat = SDL_GetGPUSwapchainTextureFormat(app_state->gpu_device, app_state->window);
  init_info.MSAASamples = SDL_GPU_SAMPLECOUNT_1;                      // Only used in multi-viewports mode.
  init_info.SwapchainComposition = SDL_GPU_SWAPCHAINCOMPOSITION_SDR;  // Only used in multi-viewports mode.
  init_info.PresentMode = SDL_GPU_PRESENTMODE_VSYNC;
  ImGui_ImplSDLGPU3_Init(&init_info);

         // Load Fonts
         // - If fonts are not explicitly loaded, Dear ImGui will select an embedded font: either AddFontDefaultVector() or AddFontDefaultBitmap().
         //   This selection is based on (style.FontSizeBase * style.FontScaleMain * style.FontScaleDpi) reaching a small threshold.
         // - You can load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
         // - If a file cannot be loaded, AddFont functions will return a nullptr. Please handle those errors in your code (e.g. use an assertion, display an error and quit).
         // - Read 'docs/FONTS.md' for more instructions and details.
         // - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use FreeType for higher quality font rendering.
         // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
         //style.FontSizeBase = 20.0f;
         //io.Fonts->AddFontDefaultVector();
         //io.Fonts->AddFontDefaultBitmap();
         //io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeui.ttf");
         //io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf");
         //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf");
         //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf");
         //ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf");
         //IM_ASSERT(font != nullptr);
  return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event) {
  AppState* app_state = (AppState*)appstate;
  ImGui_ImplSDL3_ProcessEvent(event);
  if (event->type == SDL_EVENT_QUIT)
    return SDL_APP_SUCCESS;
  if (event->type == SDL_EVENT_WINDOW_CLOSE_REQUESTED && event->window.windowID == SDL_GetWindowID(app_state->window))
    return SDL_APP_SUCCESS;
  return SDL_APP_CONTINUE;
}

static bool show_demo_window = true;
static bool show_another_window = false;
static ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

SDL_AppResult SDL_AppIterate(void *appstate) {
  AppState* app_state = (AppState*)appstate;
  if (SDL_GetWindowFlags(app_state->window) & SDL_WINDOW_MINIMIZED)
    {
      SDL_Delay(10);
      return SDL_APP_CONTINUE;
    }

         // Start the Dear ImGui frame
  ImGui_ImplSDLGPU3_NewFrame();
  ImGui_ImplSDL3_NewFrame();
  ImGui::NewFrame();

         // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
  if (show_demo_window)
    ImGui::ShowDemoWindow(&show_demo_window);

         // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
  {
    static float f = 0.0f;
    static int counter = 0;

    ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.

    ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)
    ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
    ImGui::Checkbox("Another Window", &show_another_window);

    ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
    ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

    if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
      counter++;
    ImGui::SameLine();
    ImGui::Text("counter = %d", counter);

    //ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
    ImGui::End();
  }

         // 3. Show another simple window.
  if (show_another_window)
    {
      ImGui::Begin("Another Window", &show_another_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
      ImGui::Text("Hello from another window!");
      if (ImGui::Button("Close Me"))
        show_another_window = false;
      ImGui::End();
    }

         // Rendering
  ImGui::Render();
  ImDrawData* draw_data = ImGui::GetDrawData();
  const bool is_minimized = (draw_data->DisplaySize.x <= 0.0f || draw_data->DisplaySize.y <= 0.0f);

  SDL_GPUCommandBuffer* command_buffer = SDL_AcquireGPUCommandBuffer(app_state->gpu_device); // Acquire a GPU command buffer

  SDL_GPUTexture* swapchain_texture;
  SDL_WaitAndAcquireGPUSwapchainTexture(command_buffer, app_state->window, &swapchain_texture, nullptr, nullptr); // Acquire a swapchain texture

  if (swapchain_texture != nullptr && !is_minimized)
    {
      // This is mandatory: call ImGui_ImplSDLGPU3_PrepareDrawData() to upload the vertex/index buffer!
      ImGui_ImplSDLGPU3_PrepareDrawData(draw_data, command_buffer);

             // Setup and start a render pass
      SDL_GPUColorTargetInfo target_info = {};
      target_info.texture = swapchain_texture;
      target_info.clear_color = SDL_FColor { clear_color.x, clear_color.y, clear_color.z, clear_color.w };
      target_info.load_op = SDL_GPU_LOADOP_CLEAR;
      target_info.store_op = SDL_GPU_STOREOP_STORE;
      target_info.mip_level = 0;
      target_info.layer_or_depth_plane = 0;
      target_info.cycle = false;
      SDL_GPURenderPass* render_pass = SDL_BeginGPURenderPass(command_buffer, &target_info, 1, nullptr);

             // Render ImGui
      ImGui_ImplSDLGPU3_RenderDrawData(draw_data, command_buffer, render_pass);

      SDL_EndGPURenderPass(render_pass);
    }

         // Submit the command buffer
  SDL_SubmitGPUCommandBuffer(command_buffer);
  return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void *appstate, SDL_AppResult result) {
  AppState* app_state = (AppState*)appstate;
  SDL_WaitForGPUIdle(app_state->gpu_device);
  ImGui_ImplSDL3_Shutdown();
  ImGui_ImplSDLGPU3_Shutdown();
  ImGui::DestroyContext();

  SDL_ReleaseWindowFromGPUDevice(app_state->gpu_device, app_state->window);
  SDL_DestroyGPUDevice(app_state->gpu_device);
  SDL_DestroyWindow(app_state->window);
  SDL_Quit();
}
