#pragma once

#include "config.h"

#include <windows.h>
#include <tlhelp32.h>
#include <stdarg.h>
#include <stdio.h>
#include <openvr_driver.h>

#include <format>
#include <iostream>

// Define PI for degree-to-radian conversion
#define PI 3.14159265359f

static void nullsub() {}
static __int64 nullsub_0() { return 0; }

namespace psvr2_toolkit {

  class Util {
  public:
    static bool StartsWith(const char *a, const char *b) {
      return strncmp(a, b, strlen(b)) == 0;
    }

    static bool IsRunningOnWine() {
#if !MOCK_IS_RUNNING_ON_WINE
      HMODULE hModule = GetModuleHandleW(L"ntdll.dll");
      if (!hModule) {
        return false;
      }
      return GetProcAddress(hModule, "wine_get_version") != nullptr;
#else
      return true;
#endif
    }

    static bool IsProcessRunning(DWORD dwProcessId) {
      HANDLE hSnapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
      if (hSnapshot == INVALID_HANDLE_VALUE) {
        return false;
      }

      PROCESSENTRY32W pe;
      pe.dwSize = sizeof(pe);

      if (Process32FirstW(hSnapshot, &pe)) {
        do {
          if (pe.th32ProcessID == dwProcessId) {
            CloseHandle(hSnapshot);
            return true;
          }
        } while (Process32NextW(hSnapshot, &pe));
      }

      CloseHandle(hSnapshot);
      return false;
    }


    // TEMP
    static void SetIdentity(vr::HmdMatrix34_t* pMatrix) {
        pMatrix->m[0][0] = 1.f;
        pMatrix->m[0][1] = 0.f;
        pMatrix->m[0][2] = 0.f;
        pMatrix->m[0][3] = 0.f;
        pMatrix->m[1][0] = 0.f;
        pMatrix->m[1][1] = 1.f;
        pMatrix->m[1][2] = 0.f;
        pMatrix->m[1][3] = 0.f;
        pMatrix->m[2][0] = 0.f;
        pMatrix->m[2][1] = 0.f;
        pMatrix->m[2][2] = 1.f;
        pMatrix->m[2][3] = 0.f;
    }

    // TEMP
    static void SetIdentity(vr::HmdMatrix44_t *pMatrix) {
      pMatrix->m[0][0] = 1.f;
      pMatrix->m[0][1] = 0.f;
      pMatrix->m[0][2] = 0.f;
      pMatrix->m[0][3] = 0.f;
      pMatrix->m[1][0] = 0.f;
      pMatrix->m[1][1] = 1.f;
      pMatrix->m[1][2] = 0.f;
      pMatrix->m[1][3] = 0.f;
      pMatrix->m[2][0] = 0.f;
      pMatrix->m[2][1] = 0.f;
      pMatrix->m[2][2] = 1.f;
      pMatrix->m[2][3] = 0.f;
      pMatrix->m[3][0] = 0.f;
      pMatrix->m[3][1] = 0.f;
      pMatrix->m[3][2] = 0.f;
      pMatrix->m[3][3] = 1.f;
    }

    /**
    * @brief Creates a vr::HmdMatrix34_t transform from Euler angles and a position vector.
    * @param position The position vector for the transform.
    * @param yaw Rotation around the Y axis in radians.
    * @param pitch Rotation around the X axis in radians.
    * @param roll Rotation around the Z axis in radians.
    * @return A vr::HmdMatrix34_t representing the combined transform.
    */
    static vr::HmdMatrix34_t createTransformMatrixFromEuler(const vr::HmdVector3_t& position, float yaw, float pitch, float roll) {
        vr::HmdMatrix34_t transform{};

        float cy = cosf(toRadians(yaw));
        float sy = sinf(toRadians(yaw));
        float cp = cosf(toRadians(pitch));
        float sp = sinf(toRadians(pitch));
        float cr = cosf(toRadians(roll));
        float sr = sinf(toRadians(roll));

        // Rotation matrix from Yaw (Y), Pitch (X), Roll (Z)
        transform.m[0][0] = cy * cr + sy * sp * sr;
        transform.m[0][1] = -cy * sr + sy * sp * cr;
        transform.m[0][2] = sy * cp;

        transform.m[1][0] = sr * cp;
        transform.m[1][1] = cr * cp;
        transform.m[1][2] = -sp;

        transform.m[2][0] = -sy * cr + cy * sp * sr;
        transform.m[2][1] = sy * sr + cy * sp * cr;
        transform.m[2][2] = cy * cp;

        // Add the translation part
        transform.m[0][3] = position.v[0];
        transform.m[1][3] = position.v[1];
        transform.m[2][3] = position.v[2];

        return transform;
    }

    /**
     * @brief Converts a 3x4 pose matrix to a 4x4 matrix.
     * @param mat34 The 3x4 matrix to convert.
     * @return The resulting 4x4 matrix.
     */
    static vr::HmdMatrix44_t convert34to44(const vr::HmdMatrix34_t& mat34) {
        vr::HmdMatrix44_t mat44{};
        // Copy rotation and translation
        mat44.m[0][0] = mat34.m[0][0]; mat44.m[0][1] = mat34.m[0][1]; mat44.m[0][2] = mat34.m[0][2]; mat44.m[0][3] = mat34.m[0][3];
        mat44.m[1][0] = mat34.m[1][0]; mat44.m[1][1] = mat34.m[1][1]; mat44.m[1][2] = mat34.m[1][2]; mat44.m[1][3] = mat34.m[1][3];
        mat44.m[2][0] = mat34.m[2][0]; mat44.m[2][1] = mat34.m[2][1]; mat44.m[2][2] = mat34.m[2][2]; mat44.m[2][3] = mat34.m[2][3];
        // Add the bottom row for a 4x4 matrix
        mat44.m[3][0] = 0.0f; mat44.m[3][1] = 0.0f; mat44.m[3][2] = 0.0f; mat44.m[3][3] = 1.0f;
        return mat44;
    }

    /**
     * @brief Converts a 4x4 pose matrix to a 3x4 matrix.
     * @param mat44 The 4x4 matrix to convert.
     * @return The resulting 3x4 matrix.
     */
    static vr::HmdMatrix34_t convert44to34(const vr::HmdMatrix44_t& mat44) {
      vr::HmdMatrix34_t mat34{};
      // Copy rotation and translation
      mat34.m[0][0] = mat44.m[0][0]; mat34.m[0][1] = mat44.m[0][1]; mat34.m[0][2] = mat44.m[0][2]; mat34.m[0][3] = mat44.m[0][3];
      mat34.m[1][0] = mat44.m[1][0]; mat34.m[1][1] = mat44.m[1][1]; mat34.m[1][2] = mat44.m[1][2]; mat34.m[1][3] = mat44.m[1][3];
      mat34.m[2][0] = mat44.m[2][0]; mat34.m[2][1] = mat44.m[2][1]; mat34.m[2][2] = mat44.m[2][2]; mat34.m[2][3] = mat44.m[2][3];
      return mat34;
    }


    /**
     * @brief Inverts a rigid-body transform matrix (like a camera pose).
     * This is much faster and more numerically stable than a general 4x4 matrix inverse.
     * The inverse of [R|t] is [R^T | -R^T * t]
     * @param mat The 4x4 matrix to invert.
     * @return The inverted matrix.
     */
    static vr::HmdMatrix44_t invertTransform(const vr::HmdMatrix44_t& mat) {
        vr::HmdMatrix44_t result{};
        // Transpose the 3x3 rotation part
        result.m[0][0] = mat.m[0][0]; result.m[0][1] = mat.m[1][0]; result.m[0][2] = mat.m[2][0];
        result.m[1][0] = mat.m[0][1]; result.m[1][1] = mat.m[1][1]; result.m[1][2] = mat.m[2][1];
        result.m[2][0] = mat.m[0][2]; result.m[2][1] = mat.m[1][2]; result.m[2][2] = mat.m[2][2];

        // Calculate -R^T * t
        float x = mat.m[0][3];
        float y = mat.m[1][3];
        float z = mat.m[2][3];
        result.m[0][3] = -(result.m[0][0] * x + result.m[0][1] * y + result.m[0][2] * z);
        result.m[1][3] = -(result.m[1][0] * x + result.m[1][1] * y + result.m[1][2] * z);
        result.m[2][3] = -(result.m[2][0] * x + result.m[2][1] * y + result.m[2][2] * z);

        // Bottom row
        result.m[3][0] = 0.0f; result.m[3][1] = 0.0f; result.m[3][2] = 0.0f; result.m[3][3] = 1.0f;

        return result;
    }

    /**
     * @brief Multiplies two 4x4 matrices (A * B).
     * @param matA The first matrix.
     * @param matB The second matrix.
     * @return The resulting matrix.
     */
    static vr::HmdMatrix44_t multiplyMatrix(const vr::HmdMatrix44_t& matA, const vr::HmdMatrix44_t& matB) {
        vr::HmdMatrix44_t result{};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                result.m[i][j] = 0;
                for (int k = 0; k < 4; k++) {
                    result.m[i][j] += matA.m[i][k] * matB.m[k][j];
                }
            }
        }
        return result;
    }

    /**
     * @brief Converts an angle from degrees to radians.
     * @param degrees The angle in degrees.
     * @return The angle in radians.
     */
    static float toRadians(float degrees) {
        return degrees * PI / 180.0f;
    }

    /**
     * @brief Creates a perspective projection matrix with an asymmetric frustum for OpenVR.
     *
     * This function generates a projection matrix based on field-of-view angles
     * for each side of the viewing frustum, along with near and far clipping planes.
     * The resulting matrix is a vr::HmdMatrix44_t for a right-handed coordinate system.
     *
     * @param nearVal The distance to the near clipping plane. Must be positive.
     * @param farVal The distance to the far clipping plane. Must be positive and greater than nearVal.
     * @param fovLeftDeg The horizontal field of view angle to the left, in degrees.
     * @param fovRightDeg The horizontal field of view angle to the right, in degrees.
     * @param fovTopDeg The vertical field of view angle to the top, in degrees.
     * @param fovBottomDeg The vertical field of view angle to the bottom, in degrees.
     * @return The calculated vr::HmdMatrix44_t projection matrix.
     */
    static vr::HmdMatrix44_t createProjectionMatrix(float nearVal, float farVal, float left, float right, float top, float bottom) {
        vr::HmdMatrix44_t mat{};

        float const r_l = right - left;
        float const t_b = top - bottom;
        float const f_n = farVal - nearVal;

        // Row 0
        mat.m[0][0] = (2.0f * nearVal) / r_l;
        mat.m[0][1] = 0.0f;
        mat.m[0][2] = (right + left) / r_l;
        mat.m[0][3] = 0.0f;

        // Row 1
        mat.m[1][0] = 0.0f;
        mat.m[1][1] = (2.0f * nearVal) / t_b;
        mat.m[1][2] = (top + bottom) / t_b;
        mat.m[1][3] = 0.0f;

        // Row 2
        mat.m[2][0] = 0.0f;
        mat.m[2][1] = 0.0f;
        mat.m[2][2] = -(farVal + nearVal) / f_n;
        mat.m[2][3] = -(2.0f * farVal * nearVal) / f_n;

        // Row 3
        mat.m[3][0] = 0.0f;
        mat.m[3][1] = 0.0f;
        mat.m[3][2] = -1.0f;
        mat.m[3][3] = 0.0f;

        return mat;
    }

    static vr::HmdQuaternion_t EulerToQuaternion(double yaw, double pitch, double roll) {
      double cy = cos(yaw * 0.5);
      double sy = sin(yaw * 0.5);
      double cp = cos(pitch * 0.5);
      double sp = sin(pitch * 0.5);
      double cr = cos(roll * 0.5);
      double sr = sin(roll * 0.5);
      return {
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy
      };
    }

    template <typename... Args>
    static void DriverLog(const char *format, const Args&... args) {
      std::string message = std::vformat(std::string_view(format), std::make_format_args(args...));
      vr::VRDriverLog()->Log(message.c_str());
    }

    static std::string WideStringToUTF8(const std::wstring &wideStr) {
      if (wideStr.empty()) {
        return std::string();
      }
      int sizeNeeded = WideCharToMultiByte(CP_UTF8, 0, wideStr.data(), (int)wideStr.size(), NULL, 0, NULL, NULL);
      std::string utf8Str(sizeNeeded, 0);
      WideCharToMultiByte(CP_UTF8, 0, wideStr.data(), (int)wideStr.size(), &utf8Str[0], sizeNeeded, NULL, NULL);
      return utf8Str;
    }
  };

} // psvr2_toolkit
