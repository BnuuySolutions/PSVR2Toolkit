#pragma once

#include <openvr_driver.h>

#include <vector>

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

      // Extract the inner points in their native sweep order, ignoring duplicates
      std::vector<vr::HmdVector2_t> perimeter;
      for (const auto& v : standardMesh) {
        float dx = v.v[0] - centerX;
        float dy = v.v[1] - centerY;

        // Skip the far outer corners
        if ((dx * dx + dy * dy) > thresholdSq) {
          continue; 
        }

        // Keep sequential unique points
        if (perimeter.empty() || 
          perimeter.back().v[0] != v.v[0] || 
          perimeter.back().v[1] != v.v[1]) {
          perimeter.push_back(v);
        }
      }

      // Pop the last vertex if it closes the loop perfectly
      if (perimeter.size() > 1 && 
        perimeter.front().v[0] == perimeter.back().v[0] && 
        perimeter.front().v[1] == perimeter.back().v[1]) {
        perimeter.pop_back();
      }

      return perimeter;
    }
  };

}
