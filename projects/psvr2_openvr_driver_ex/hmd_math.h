#pragma once

#include <openvr_driver.h>

#include <algorithm>
#include <cmath>
#include <vector>

#define PI 3.14159265359f

namespace psvr2_toolkit {

  class HmdMath {
  public:
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

    static vr::HmdQuaternion_t QuaternionMultiply(const vr::HmdQuaternion_t &a, const vr::HmdQuaternion_t &b) {
      return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
      };
    }

    static vr::HmdQuaternion_t QuaternionInverse(const vr::HmdQuaternion_t &q) {
      double normSq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
      return { q.w / normSq, -q.x / normSq, -q.y / normSq, -q.z / normSq };
    }

    static vr::HmdVector3d_t RotateVectorByQuaternion(const vr::HmdVector3d_t &v, const vr::HmdQuaternion_t &q) {
      vr::HmdQuaternion_t vQuat = { 0.0, v.v[0], v.v[1], v.v[2] };
      vr::HmdQuaternion_t qv = QuaternionMultiply(q, vQuat);
      vr::HmdQuaternion_t qInv = QuaternionInverse(q);
      vr::HmdQuaternion_t result = QuaternionMultiply(qv, qInv);
      return { result.x, result.y, result.z };
    }

    static std::vector<vr::HmdVector2_t> ExtractInnerHAMPerimeter(const std::vector<vr::HmdVector2_t>& standardMesh) {
      if (standardMesh.empty()) return {};

      // Find the center of the mesh
      float minX = 9999.0f, maxX = -9999.0f, minY = 9999.0f, maxY = -9999.0f;
      for (const auto& v : standardMesh) {
          if (v.v[0] < minX) minX = v.v[0];
          if (v.v[0] > maxX) maxX = v.v[0];
          if (v.v[1] < minY) minY = v.v[1];
          if (v.v[1] > maxY) maxY = v.v[1];
      }
      
      float centerX = (minX + maxX) / 2.0f;
      float centerY = (minY + maxY) / 2.0f;

      // Get the distance to the furthest outer corner
      float maxDistSq = 0.0f;
      for (const auto& v : standardMesh) {
        float dx = v.v[0] - centerX;
        float dy = v.v[1] - centerY;
        float distSq = (dx * dx) + (dy * dy);
        if (distSq > maxDistSq) maxDistSq = distSq;
      }

      // Set a dynamic threshold (the inner circle is much closer to the center than the corners)
      float thresholdSq = maxDistSq * 0.9f;
      const float epsilon = 1e-3f;

      // Extract the inner points
      std::vector<vr::HmdVector2_t> perimeter;
      for (const auto& v : standardMesh) {
        float dx = v.v[0] - centerX;
        float dy = v.v[1] - centerY;

        // Skip the far outer corners
        if ((dx * dx + dy * dy) > thresholdSq) {
          continue; 
        }

        perimeter.push_back(v);
      }

      // Sort by angle from center
      std::sort(perimeter.begin(), perimeter.end(), [centerX, centerY](const vr::HmdVector2_t& a, const vr::HmdVector2_t& b) {
        float angleA = atan2(a.v[1] - centerY, a.v[0] - centerX);
        float angleB = atan2(b.v[1] - centerY, b.v[0] - centerX);
        return angleA < angleB;
      });

      // Remove duplicates now that they are adjacent
      perimeter.erase(std::unique(perimeter.begin(), perimeter.end(), [epsilon](const vr::HmdVector2_t& a, const vr::HmdVector2_t& b) {
        return std::abs(a.v[0] - b.v[0]) <= epsilon && std::abs(a.v[1] - b.v[1]) <= epsilon;
      }), perimeter.end());

      // Close the loop
      perimeter.push_back(perimeter.at(0));

      return perimeter;
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
  };

}
