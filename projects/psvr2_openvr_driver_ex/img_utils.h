#pragma once
#include <cstdint>
#include <cstring>
#include <math.h>
#include <vector>

#include <d3d11.h>
#include <d3dcompiler.h>
#include <stdexcept>
#include <string>
#include <cmath>
#include <algorithm>

#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "d3dcompiler.lib")

// Helper macro to safely release COM objects
#define SAFE_RELEASE(p) { if(p) { (p)->Release(); (p) = nullptr; } }

class BC4_to_NV12_Converter {
public:
  // width and height are for a SINGLE eye. The output will be 2 * width.
  BC4_to_NV12_Converter(uint32_t srcW, uint32_t dstW, uint32_t h)
    : m_srcWidth(srcW), m_dstWidth(dstW), m_height(h)
  {
    initialize_device_and_shader();

    HRESULT hr;
    D3D11_TEXTURE2D_DESC desc{};
    desc.Width = m_srcWidth;
    desc.Height = m_height;
    desc.MipLevels = 1;
    desc.ArraySize = 2; // Array size 2 for Left and Right eyes
    desc.SampleDesc.Count = 1;
    desc.Format = DXGI_FORMAT_BC4_UNORM;
    desc.Usage = D3D11_USAGE_DEFAULT;
    desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
    desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;

    hr = m_device->CreateTexture2D(&desc, nullptr, &m_texInputArray);
    if (FAILED(hr)) throw std::runtime_error("Failed to create input texture array.");

    // SRV for the Texture Array
    hr = m_device->CreateShaderResourceView(m_texInputArray, nullptr, &m_srvInput);

    // Output textures are DOUBLE width for Side-by-Side
    desc.ArraySize = 1;
    desc.Width = dstW * 2;
    desc.Usage = D3D11_USAGE_DEFAULT;
    desc.BindFlags = D3D11_BIND_UNORDERED_ACCESS;
    desc.CPUAccessFlags = 0;
    desc.Format = DXGI_FORMAT_R8_UNORM;

    hr = m_device->CreateTexture2D(&desc, nullptr, &m_texOutputY);
    if (FAILED(hr)) throw std::runtime_error("Failed to create output texture.");
    hr = m_device->CreateUnorderedAccessView(m_texOutputY, nullptr, &m_uavOutputY);
    if (FAILED(hr)) throw std::runtime_error("Failed to create UAV for output texture.");

    // Staging for readback (Full SBS size)
    desc.Usage = D3D11_USAGE_STAGING;
    desc.BindFlags = 0;
    desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
    hr = m_device->CreateTexture2D(&desc, nullptr, &m_stagingTexY);
  }

  void convert(const void* leftData, const void* rightData, void* nv12Data) {
    if (!m_context || !m_texInputArray) return;

    // BC4 Row Pitch calculation: (Width / 4 blocks) * 8 bytes per block
    const uint32_t bc4RowPitch = (m_srcWidth / 4) * 8;

    // --- Upload Left Eye to Array Index 0 ---
    m_context->UpdateSubresource(m_texInputArray, D3D11CalcSubresource(0, 0, 1), nullptr, leftData, bc4RowPitch, 0);

    // --- Upload Right Eye to Array Index 1 ---
    m_context->UpdateSubresource(m_texInputArray, D3D11CalcSubresource(0, 1, 1), nullptr, rightData, bc4RowPitch, 0);

    // Set up Pipeline
    m_context->CSSetShader(m_shader, nullptr, 0);
    m_context->CSSetShaderResources(0, 1, &m_srvInput);

    // We only need the Y-plane UAV here; UV is handled separately or in shader
    ID3D11UnorderedAccessView* uavs[] = { m_uavOutputY };
    m_context->CSSetUnorderedAccessViews(0, 1, uavs, nullptr);

    // Dispatch: We are filling a side-by-side texture (2 * m_width)
    m_context->Dispatch((m_srcWidth * 2 + 7) / 8, (m_height + 7) / 8, 1);

    // Unbind UAVs to allow CopyResource
    ID3D11UnorderedAccessView* nullUAVs[] = { nullptr };
    m_context->CSSetUnorderedAccessViews(0, 1, nullUAVs, nullptr);

    // Download the combined SBS Y-plane
    m_context->CopyResource(m_stagingTexY, m_texOutputY);

    D3D11_MAPPED_SUBRESOURCE mappedY{};
    HRESULT hr = m_context->Map(m_stagingTexY, 0, D3D11_MAP_READ, 0, &mappedY);
    if (SUCCEEDED(hr)) {
      const uint32_t sbsWidth = m_dstWidth * 2;
      if (mappedY.RowPitch == sbsWidth) {
        memcpy(nv12Data, mappedY.pData, sbsWidth * m_height);
      }
      else {
        for (uint32_t i = 0; i < m_height; ++i) {
          uint8_t* pSrc = static_cast<uint8_t*>(mappedY.pData) + i * mappedY.RowPitch;
          uint8_t* pDst = static_cast<uint8_t*>(nv12Data) + i * sbsWidth;
          memcpy(pDst, pSrc, sbsWidth);
        }
      }
      m_context->Unmap(m_stagingTexY, 0);
    }

    // Fill UV-plane with neutral grey (128)
    // The UV plane starts after the full SBS Y-plane
    const size_t sbsYSize = (m_dstWidth * 2) * m_height;
    const size_t sbsUVSize = sbsYSize / 2;
    uint8_t* uvPlaneStart = static_cast<uint8_t*>(nv12Data) + sbsYSize;
    memset(uvPlaneStart, 128, sbsUVSize);
  }

private:
  void initialize_device_and_shader() {
    // Device creation logic...

    const char* shaderSource = R"(
        Texture2DArray<float> g_InputBC4 : register(t0);
        RWTexture2D<unorm float> g_OutputY : register(u0);

        [numthreads(8, 8, 1)]
        void CSMain(uint3 DTid : SV_DispatchThreadID) {
          uint sbsWidth, height;
          g_OutputY.GetDimensions(sbsWidth, height);
          if (DTid.x >= sbsWidth || DTid.y >= height) return;

          uint singleWidth = sbsWidth / 2;
          uint eyeIndex = (DTid.x < singleWidth) ? 0 : 1;
          uint localX = DTid.x % singleWidth;

          // Load from the specific eye in the texture array
          float luma = g_InputBC4.Load(uint4(localX, DTid.y, eyeIndex, 0)).r;
                
          // Your specific luma curve
          g_OutputY[DTid.xy] = (pow(luma, 0.75f) * 2.0f) - 0.135f;
        }
    )";

    HRESULT hr = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, nullptr, 0, D3D11_SDK_VERSION, &m_device, nullptr, &m_context);
    if (FAILED(hr)) throw std::runtime_error("Failed to create D3D11 device.");

    ID3DBlob* csBlob = nullptr;
    ID3DBlob* errorBlob = nullptr;
    hr = D3DCompile(shaderSource, strlen(shaderSource), nullptr, nullptr, nullptr, "CSMain", "cs_5_0", 0, 0, &csBlob, &errorBlob);
    if (FAILED(hr)) {
      std::string errMsg = "Shader compilation failed: ";
      if (errorBlob) {
        errMsg += (char*)errorBlob->GetBufferPointer();
        errorBlob->Release();
      }
      throw std::runtime_error(errMsg);
    }
    if (errorBlob) SAFE_RELEASE(errorBlob);

    hr = m_device->CreateComputeShader(csBlob->GetBufferPointer(), csBlob->GetBufferSize(), nullptr, &m_shader);
    SAFE_RELEASE(csBlob);
    if (FAILED(hr)) throw std::runtime_error("Failed to create compute shader.");
  }

  uint32_t m_srcWidth;
  uint32_t m_dstWidth;
  uint32_t m_height;
  ID3D11Device* m_device = nullptr;
  ID3D11DeviceContext* m_context = nullptr;
  ID3D11ComputeShader* m_shader = nullptr;
  ID3D11Texture2D* m_texInputArray = nullptr;
  ID3D11ShaderResourceView* m_srvInput = nullptr;
  ID3D11Texture2D* m_texOutputY = nullptr;
  ID3D11UnorderedAccessView* m_uavOutputY = nullptr;
  ID3D11Texture2D* m_texOutputUV = nullptr;
  ID3D11UnorderedAccessView* m_uavOutputUV = nullptr;
  ID3D11Texture2D* m_stagingTexY = nullptr;
};

class BC4_to_NV12_Converter_CPU {
public:
  // sourceWidth = 1024, targetWidth = 1016, height = 1016
  BC4_to_NV12_Converter_CPU(uint32_t srcW, uint32_t dstW, uint32_t h)
    : m_srcWidth(srcW), m_dstWidth(dstW), m_height(h) {
    generateLut();
  }

  void convert(const void* leftData, const void* rightData, void* nv12Data) {
    const uint32_t sbsWidth = m_dstWidth * 2;
    uint8_t* outY = static_cast<uint8_t*>(nv12Data);

    // 1. Process Left Eye (writes to x=0 to 1015)
    processEye(static_cast<const uint8_t*>(leftData), outY, 0);

    // 2. Process Right Eye (writes to x=1016 to 2031)
    processEye(static_cast<const uint8_t*>(rightData), outY, m_dstWidth);

    // 3. UV-plane (Neutral Grey)
    const size_t sbsYSize = sbsWidth * m_height;
    const size_t sbsUVSize = sbsYSize / 2;
    std::memset(outY + sbsYSize, 128, sbsUVSize);
  }

private:
  void processEye(const uint8_t* bc4Data, uint8_t* outY, uint32_t xOffset) {
    const uint32_t sbsWidth = m_dstWidth * 2;
    const uint32_t blocksX = m_srcWidth / 4; // 1024 / 4 = 256 blocks
    const uint32_t blocksY = m_height / 4;   // 1016 / 4 = 254 blocks

    for (uint32_t by = 0; by < blocksY; ++by) {
      for (uint32_t bx = 0; bx < blocksX; ++bx) {
        // Skip processing blocks that start beyond our 1016 target width
        if (bx * 4 >= m_dstWidth) continue;

        const uint8_t* block = bc4Data + (by * blocksX + bx) * 8;
        uint8_t endpoints[8];
        endpoints[0] = block[0];
        endpoints[1] = block[1];

        // BC4 Interpolation
        if (endpoints[0] > endpoints[1]) {
          for (int i = 0; i < 6; ++i)
            endpoints[i + 2] = (uint8_t)(((6 - i) * endpoints[0] + (1 + i) * endpoints[1]) / 7);
        }
        else {
          for (int i = 0; i < 4; ++i)
            endpoints[i + 2] = (uint8_t)(((4 - i) * endpoints[0] + (1 + i) * endpoints[1]) / 5);
          endpoints[6] = 0; endpoints[7] = 255;
        }

        uint64_t indices = 0;
        std::memcpy(&indices, block + 2, 6);

        for (uint32_t ly = 0; ly < 4; ++ly) {
          for (uint32_t lx = 0; lx < 4; ++lx) {
            uint32_t outX = (bx * 4) + lx;
            // Final crop check for pixels within a block that straddles the boundary
            if (outX >= m_dstWidth) continue;

            uint32_t pixelIdx = ly * 4 + lx;
            uint8_t index = (indices >> (pixelIdx * 3)) & 0x7;

            // Use LUT for the math: (pow(luma, 0.75) * 2.0) - 0.135
            outY[(by * 4 + ly) * sbsWidth + (xOffset + outX)] = m_lut[endpoints[index]];
          }
        }
      }
    }
  }

  void generateLut() {
    for (int i = 0; i < 256; ++i) {
      float luma = i / 255.0f;
      float processed = (std::pow(luma, 0.75f) * 2.0f) - 0.135f;
      m_lut[i] = (uint8_t)(std::max(0.0f, std::min(1.0f, processed)) * 255.0f);
    }
  }

  uint32_t m_srcWidth, m_dstWidth, m_height;
  uint8_t m_lut[256];
};


typedef struct {
  double params[20];
} DistortionParameters;

// Forward declaration for the matrix multiplication helper
void multiply_3x3_matrices(float* out, const float* a, const float* b);

/**
 * @brief Computes a 3x3 rotation matrix from an axis-angle representation.
 *
 * This function implements Rodrigues' rotation formula. Note that it effectively
 * computes the rotation for the *negative* of the provided angle, which is
 * equivalent to the inverse (or transpose) of the standard rotation matrix.
 * This is common when transforming coordinates from a rotated frame back to a
 * parent frame.
 *
 * @param axis_angle A 4-element float array: {axis_x, axis_y, axis_z, angle_in_radians}.
 * @param out_matrix A 9-element float array to store the resulting row-major 3x3 matrix.
 */
void computeRotationMatrixFromAxisAngle(const float* axis_angle, float* out_matrix) {
  float x = axis_angle[0];
  float y = axis_angle[1];
  float z = axis_angle[2];
  float angle = axis_angle[3];

  float s = sinf(angle);
  float c = cosf(angle);
  float t = 1.0f - c;

  // The logic from the assembly matches Rodrigues' formula for a negative angle.
  // R[0][1] = t*x*y - s*z  (standard formula)
  // R[0][1] = t*x*y + s*z  (this implementation, from s -> -s)
  out_matrix[0] = t * x * x + c;
  out_matrix[1] = t * x * y + s * z;
  out_matrix[2] = t * x * z - s * y;

  out_matrix[3] = t * x * y - s * z;
  out_matrix[4] = t * y * y + c;
  out_matrix[5] = t * y * z + s * x;

  out_matrix[6] = t * x * z + s * y;
  out_matrix[7] = t * y * z - s * x;
  out_matrix[8] = t * z * z + c;
}

/**
 * @brief Applies a complex lens distortion and rotation model to a 2D point.
 *
 * @param out_xy A float pointer to store the resulting 2D point {x, y}.
 * @param d_params A pointer to the structure containing distortion coefficients.
 * @param x_in The input X coordinate.
 * @param y_in The input Y coordinate.
 */
void applyDistortionTransform(float* out_xy, const DistortionParameters* d_params, float x_in, float y_in) {
  // 1. Calculate radial and tangential distortion components
  float x_sq = x_in * x_in;
  float y_sq = y_in * y_in;
  float r_sq = x_sq + y_sq;
  float two_xy = 2.0f * x_in * y_in;

  // Calculate high-order powers of the radius
  float r_sq_p4 = r_sq * r_sq * r_sq * r_sq; // r^8
  float r_sq_p5 = r_sq_p4 * r_sq;            // r^10

  // 2. Calculate the main rational distortion factor.
  // This is a ratio of two high-order polynomials in r^2.
  // Numerator coefficients
  const float k1 = (float)d_params->params[0];
  const float k2 = (float)d_params->params[1];
  const float k3 = (float)d_params->params[4];
  const float k4 = (float)d_params->params[14];
  const float k5 = (float)d_params->params[15];
  const float k6 = (float)d_params->params[16];
  // Denominator coefficients
  const float d1 = (float)d_params->params[5];
  const float d2 = (float)d_params->params[6];
  const float d3 = (float)d_params->params[7];
  const float d4 = (float)d_params->params[17];
  const float d5 = (float)d_params->params[18];
  const float d6 = (float)d_params->params[19];

  float numerator = (((k3 * r_sq + k2) * r_sq + k1) * r_sq) + k6 * r_sq_p5 * r_sq + k5 * r_sq_p5 + k4 * r_sq_p4 + 1.0f;
  float denominator = (((d3 * r_sq + d2) * r_sq + d1) * r_sq) + d6 * r_sq_p5 * r_sq + d5 * r_sq_p5 + d4 * r_sq_p4 + 1.0f;
  float rational_factor = numerator / denominator;

  // 3. Calculate the intermediate distorted coordinates (x', y')
  // These include tangential, polynomial, and the rational factor terms.
  // Tangential coefficients
  const float p1 = (float)d_params->params[2];
  const float p2 = (float)d_params->params[3];
  // Other polynomial coefficients
  const float c1 = (float)d_params->params[8];
  const float c2 = (float)d_params->params[9];
  const float c3 = (float)d_params->params[10];
  const float c4 = (float)d_params->params[11];

  // Note: The assembly uses the calculated rational_factor to modify the original coordinates.
  // This is a non-standard distortion model.
  float x_distorted = ((c2 * r_sq + c1) * r_sq) + (2.0f * x_sq + r_sq) * p2 + two_xy * p1 + rational_factor * x_in;
  float y_distorted = ((c4 * r_sq + c3) * r_sq) + (2.0f * y_sq + r_sq) * p1 + two_xy * p2 + rational_factor * y_in;

  // 4. Create rotation matrices for X and Y axes from the parameters
  float rot_x_angle = (float)d_params->params[12];
  float rot_y_angle = (float)d_params->params[13];

  float rot_x_axis_angle[] = { 1.0f, 0.0f, 0.0f, rot_x_angle };
  float rot_y_axis_angle[] = { 0.0f, 1.0f, 0.0f, rot_y_angle };

  float R_x[9], R_y[9], R_final[9];
  computeRotationMatrixFromAxisAngle(rot_x_axis_angle, R_x);
  computeRotationMatrixFromAxisAngle(rot_y_axis_angle, R_y);

  // 5. Combine rotations (Order is Y then X: R = Ry * Rx)
  // This rotates a point first around X, then around Y.
  multiply_3x3_matrices(R_final, R_y, R_x);

  // 6. Apply the 3D rotation to the distorted point lifted into 3D space (x', y', 1)
  // V_rot = R_final * [x_distorted, y_distorted, 1.0]^T
  float x_rot = R_final[0] * x_distorted + R_final[1] * y_distorted + R_final[2];
  float y_rot = R_final[3] * x_distorted + R_final[4] * y_distorted + R_final[5];
  float z_rot = R_final[6] * x_distorted + R_final[7] * y_distorted + R_final[8];

  // 7. Project the 3D point back to 2D via perspective division
  out_xy[0] = x_rot / z_rot;
  out_xy[1] = y_rot / z_rot;
}

/**
 * @brief Helper function to multiply two 3x3 row-major matrices (C = A * B).
 * @param out The output matrix C.
 * @param a The first matrix A.
 * @param b The second matrix B.
 */
void multiply_3x3_matrices(float* out, const float* a, const float* b) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out[i * 3 + j] = a[i * 3 + 0] * b[0 * 3 + j] +
        a[i * 3 + 1] * b[1 * 3 + j] +
        a[i * 3 + 2] * b[2 * 3 + j];
    }
  }
}

/**
 * @brief Creates a checkerboard pattern image in NV12 format.
 *
 * This function generates a grayscale checkerboard. The color information (chroma)
 * is set to neutral (128). The checkerboard pattern is created in the
 * luminance (Y) plane.
 *
 * @param nv12_data Pointer to the destination buffer for the NV12 data.
 * This buffer must be pre-allocated with a size of
 * (width * height * 3 / 2) bytes.
 * @param width The width of the image in pixels. Should be a multiple of 2.
 * @param height The height of the image in pixels. Should be a multiple of 2.
 * @param square_size The side length of each square in the checkerboard, in pixels.
 * @param y_value1 The 8-bit luminance (grayscale) value for the first color (e.g., 0 for black).
 * @param y_value2 The 8-bit luminance (grayscale) value for the second color (e.g., 255 for white).
 * @return True on success, false on failure (e.g., invalid dimensions).
 */
bool create_checkerboard_nv12(uint8_t* nv12_data, int width, int height, int square_size, uint8_t y_value1, uint8_t y_value2) {
  // Validate dimensions for NV12 format (chroma planes are half height/width)
  if (width % 2 != 0 || height % 2 != 0) {
    // Width and height must be multiples of 2 for NV12.
    return false;
  }
  if (square_size <= 0) {
    // Square size must be positive.
    return false;
  }

  // --- 1. Create the Y (Luminance) Plane ---
  uint8_t* y_plane = nv12_data;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      // Determine which checkerboard square this pixel belongs to
      int square_x = x / square_size;
      int square_y = y / square_size;

      // Assign color based on the sum of square coordinates (even or odd)
      if ((square_x + square_y) % 2 == 0) {
        y_plane[y * width + x] = y_value1;
      }
      else {
        y_plane[y * width + x] = y_value2;
      }
    }
  }

  // --- 2. Create the UV (Chrominance) Plane ---
  // The UV plane starts immediately after the Y plane.
  uint8_t* uv_plane = nv12_data + (width * height);

  // The size of the UV plane is half the size of the Y plane.
  size_t uv_plane_size = (width * height) / 2;

  // Fill the UV plane with 128. This represents a neutral gray color (no chroma),
  // which is appropriate for a grayscale checkerboard.
  memset(uv_plane, 128, uv_plane_size);

  return true;
}