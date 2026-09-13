#pragma once

#include <stdint.h>
#include <stddef.h>

#include "hmd2_gaze.h"

// Wire format for the headset's "VD" general-data payloads, delivered on the CaesarUsbThreadGenData
// pipe. A payload is a sequence of self-describing blocks; each block carries a table of items, and
// item id 0x3 holds the gaze calibration blob.

#pragma pack(push, 1)
typedef struct Hmd2GenDataHeader {
  char magic[2];
  uint16_t minSize;
  uint32_t size;
  uint16_t numItems;
  uint8_t padding[22];
} Hmd2GenDataHeader;

typedef struct Hmd2GenDataItem {
  uint16_t id;
  uint16_t unk1;
  uint32_t size;
  uint32_t offset;
} Hmd2GenDataItem;

typedef struct Hmd2GazeCalibHeader {
  char magic[2];
  uint16_t struct_version;
  uint16_t data_version;
  uint16_t data_id;
  uint32_t payload_size;
  uint32_t calib_id;
  hmd2_gaze_enabled_eye_t calib_eye;
  uint8_t padding[12];
} Hmd2GazeCalibHeader;
#pragma pack(pop)

static_assert(sizeof(Hmd2GenDataHeader) == 32, "Size of Hmd2GenDataHeader is not 32 bytes!");
static_assert(sizeof(Hmd2GenDataItem) == 12, "Size of Hmd2GenDataItem is not 12 bytes!");

constexpr uint16_t k_hmd2GenDataBlockGranularity = 0x200;
constexpr uint16_t k_hmd2GenDataItemIdGazeCalib = 0x3;


constexpr uint32_t k_hmd2GazeCalibMaxBlobSize = 0x10000;

// Filename the gaze calibration blob is persisted under, in the SteamVR user config directory.
#define HMD2_GAZE_CALIB_BLOB_FILENAME "gaze_calibration_blob.bin"

// Does this look like a calibration blob worth sending to the headset?
inline bool Hmd2IsGazeCalibBlobValid(const char *data, uint32_t size) {
  if (data == nullptr || size < sizeof(Hmd2GazeCalibHeader) || size > k_hmd2GazeCalibMaxBlobSize) {
    return false;
  }

  const Hmd2GazeCalibHeader *header = reinterpret_cast<const Hmd2GazeCalibHeader *>(data);

  // Same acceptance test the save path applies, so we never upload something we would not have stored.
  return header->calib_id != 0 && header->calib_eye == HMD2_GAZE_ENABLED_EYE_BOTH;
}

template <typename OnGazeCalibItem> inline int Hmd2ParseGenData(char *buffer, uint32_t bufferSize, OnGazeCalibItem &&onGazeCalibItem) {
  uint32_t remaining = bufferSize;

  while (remaining > 0) {
    if (remaining < k_hmd2GenDataBlockGranularity) {
      return -1;
    }

    const Hmd2GenDataHeader *header = reinterpret_cast<const Hmd2GenDataHeader *>(buffer);
    if (header->magic[0] != 'V' || header->magic[1] != 'D' || header->minSize != k_hmd2GenDataBlockGranularity || header->size < sizeof(Hmd2GenDataHeader) ||
        header->size > remaining) {
      return -1;
    }

    const uint32_t itemTableBytes = static_cast<uint32_t>(header->numItems) * static_cast<uint32_t>(sizeof(Hmd2GenDataItem));
    if (itemTableBytes > header->size - sizeof(Hmd2GenDataHeader)) {
      return -1;
    }

    const Hmd2GenDataItem *items = reinterpret_cast<const Hmd2GenDataItem *>(buffer + sizeof(Hmd2GenDataHeader));

    for (uint16_t i = 0; i < header->numItems; ++i) {
      const Hmd2GenDataItem *item = &items[i];

      if (item->offset > header->size || item->size > header->size - item->offset) {
        return -1;
      }

      if (item->id == k_hmd2GenDataItemIdGazeCalib) {
        onGazeCalibItem(buffer + item->offset, item->size);
      }
    }

    buffer += header->size;
    remaining -= header->size;
  }

  return 0;
}
