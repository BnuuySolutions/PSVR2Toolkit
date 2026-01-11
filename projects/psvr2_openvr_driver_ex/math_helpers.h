#pragma once

#include <cstdint>
#include <corecrt_math.h>
#include <corecrt_math_defines.h>
#include <set>
#include <stdexcept>

namespace psvr2_toolkit {
    constexpr uint32_t k_unSenseUnitsPerMicrosecond = 3;

    // The device time that's passed in is already divided by 3.
    // This is so that all calcuations work within this bound.
    const uint32_t k_unDeviceTimestampMax = 0xFFFFFFFF / k_unSenseUnitsPerMicrosecond;

    // The total number of unique values in the device's clock cycle (its modulus).
    const uint32_t k_unDeviceTimestampModulus = k_unDeviceTimestampMax + 1;

    static double Clamp(double value, double min, double max)
    {
      return value < min ? min : value > max ? max : value;
    }

    static int8_t ClampedAdd(int8_t a, int8_t b)
    {
      // The result is clamped to the range of int8_t
      int16_t result = static_cast<int16_t>(a) + static_cast<int16_t>(b);
      if (result > INT8_MAX)
        return INT8_MAX;
      else if (result < INT8_MIN)
        return INT8_MIN;
      else
        return static_cast<int8_t>(result);
    }

    static int8_t CosineToByte(uint32_t position, double max, double amp, double overdrive)
    {
      double cosResult = Clamp(cos((position / max) * 2 * M_PI) * overdrive, -1.0, 1.0) * amp;
        
      int8_t out = static_cast<int8_t>(cosResult);

      return out;
    }

    inline int32_t GetWraparoundDifference(uint32_t newTimestamp, uint32_t oldTimestamp)
    {
      // Ensure inputs are within the device's clock domain.
      newTimestamp %= k_unDeviceTimestampModulus;
      oldTimestamp %= k_unDeviceTimestampModulus;

      // Calculate the direct forward difference.
      // We add k_unDeviceTimestampModulus before the final modulo to prevent underflow
      // if newTimestamp < oldTimestamp, ensuring the result is always positive.
      uint32_t forwardDiff = (newTimestamp - oldTimestamp + k_unDeviceTimestampModulus) % k_unDeviceTimestampModulus;

      // The shortest path is either forward or backward. If the forward path is more
      // than halfway around the clock, the backward path is shorter.
      if (forwardDiff > k_unDeviceTimestampModulus / 2)
      {
        // The backward difference is negative.
        // This is equivalent to `forwardDiff - MODULUS`.
        return static_cast<int32_t>(forwardDiff - k_unDeviceTimestampModulus);
      }
      else
      {
        // The forward difference is the shortest path.
        return static_cast<int32_t>(forwardDiff);
      }
    }
}