// Off-device tests for Hmd2ParseGenData (common/hmd2_gen_data.h).
//
// This payload arrives straight off the USB general-data pipe, so the parser is the driver's trust
// boundary for it. The cases below pin the three bounds defects fixed in EYE_TRACKING_PLAN.md 1.2:
// a block size of zero spinning the walk forever, an item table read past the end of its block, and
// an item extent checked against the wrong base (and able to wrap a uint32_t).

#include "hmd2_gen_data.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

void Check(bool condition, const std::string &name) {
  if (condition) {
    std::printf("  pass  %s\n", name.c_str());
  } else {
    std::printf("  FAIL  %s\n", name.c_str());
    ++g_failures;
  }
}

constexpr uint32_t k_blockSize = 0x200;

struct ItemSpec {
  uint16_t id = k_hmd2GenDataItemIdGazeCalib;
  uint32_t offset = 0;
  uint32_t size = 0;
};

// Builds one 0x200-byte "VD" block with the given item table. Fields a test wants to corrupt are
// patched afterwards through HeaderAt.
std::vector<char> MakeBlock(const std::vector<ItemSpec> &items) {
  std::vector<char> block(k_blockSize, 0);

  Hmd2GenDataHeader header = {};
  header.magic[0] = 'V';
  header.magic[1] = 'D';
  header.minSize = static_cast<uint16_t>(k_hmd2GenDataBlockGranularity);
  header.size = k_blockSize;
  header.numItems = static_cast<uint16_t>(items.size());
  std::memcpy(block.data(), &header, sizeof(header));

  for (size_t i = 0; i < items.size(); ++i) {
    Hmd2GenDataItem item = {};
    item.id = items[i].id;
    item.size = items[i].size;
    item.offset = items[i].offset;
    std::memcpy(block.data() + sizeof(Hmd2GenDataHeader) + i * sizeof(Hmd2GenDataItem), &item, sizeof(item));
  }

  return block;
}

// Writes a calibration header of the shape the driver would accept into the block at the given offset.
void PlaceValidCalibPayload(std::vector<char> &block, uint32_t offset) {
  Hmd2GazeCalibHeader calib = {};
  calib.magic[0] = 'V';
  calib.magic[1] = 'C';
  calib.calib_id = 42;
  calib.calib_eye = HMD2_GAZE_ENABLED_EYE_BOTH;
  std::memcpy(block.data() + offset, &calib, sizeof(calib));
}

Hmd2GenDataHeader *HeaderAt(std::vector<char> &buffer, size_t blockIndex) {
  return reinterpret_cast<Hmd2GenDataHeader *>(buffer.data() + blockIndex * k_blockSize);
}

struct ParseOutcome {
  int result = 0;
  int calibItems = 0;
  uint32_t lastSize = 0;
};

ParseOutcome Parse(std::vector<char> &buffer) {
  ParseOutcome outcome;
  outcome.result = Hmd2ParseGenData(buffer.data(), static_cast<uint32_t>(buffer.size()), [&](char *, uint32_t size) {
    ++outcome.calibItems;
    outcome.lastSize = size;
  });
  return outcome;
}

} // namespace

int main() {
  std::printf("Hmd2ParseGenData\n");

  // --- Well-formed input ---------------------------------------------------------------------

  {
    std::vector<char> empty;
    ParseOutcome outcome = Parse(empty);
    Check(outcome.result == 0 && outcome.calibItems == 0, "empty payload parses as a no-op");
  }

  {
    std::vector<char> buffer = MakeBlock({{k_hmd2GenDataItemIdGazeCalib, 0x100, 0x40}});
    PlaceValidCalibPayload(buffer, 0x100);
    ParseOutcome outcome = Parse(buffer);
    Check(outcome.result == 0 && outcome.calibItems == 1 && outcome.lastSize == 0x40, "single block with one calibration item");
  }

  {
    std::vector<char> buffer = MakeBlock({{k_hmd2GenDataItemIdGazeCalib, 0x100, 0x40}});
    std::vector<char> second = MakeBlock({{k_hmd2GenDataItemIdGazeCalib, 0x120, 0x30}});
    buffer.insert(buffer.end(), second.begin(), second.end());
    ParseOutcome outcome = Parse(buffer);
    Check(outcome.result == 0 && outcome.calibItems == 2, "two blocks both walked");
  }

  {
    std::vector<char> buffer = MakeBlock({{0x1, 0x100, 0x40}});
    ParseOutcome outcome = Parse(buffer);
    Check(outcome.result == 0 && outcome.calibItems == 0, "non-calibration item ids are skipped");
  }

  // --- Forward progress ----------------------------------------------------------------------
  // A zero-size block previously advanced the walk by nothing, hanging the USB data thread.

  {
    std::vector<char> buffer = MakeBlock({});
    HeaderAt(buffer, 0)->size = 0;
    ParseOutcome outcome = Parse(buffer);
    Check(outcome.result == -1, "block with size 0 is rejected (does not hang)");
  }

  {
    std::vector<char> buffer = MakeBlock({});
    HeaderAt(buffer, 0)->size = sizeof(Hmd2GenDataHeader) - 1;
    ParseOutcome outcome = Parse(buffer);
    Check(outcome.result == -1, "block too small to contain its own header is rejected");
  }

  // --- Item table bounds ---------------------------------------------------------------------
  // numItems was previously trusted, so the table could be read far past the end of its block.

  {
    std::vector<char> buffer = MakeBlock({});
    HeaderAt(buffer, 0)->numItems = 0xFFFF;
    ParseOutcome outcome = Parse(buffer);
    Check(outcome.result == -1, "item table larger than its block is rejected");
  }

  {
    // (0x200 - 32) / 12 == 40 entries fit exactly; 41 does not.
    std::vector<char> buffer = MakeBlock({});
    HeaderAt(buffer, 0)->numItems = 41;
    ParseOutcome outcome = Parse(buffer);
    Check(outcome.result == -1, "item table one entry past the block end is rejected");
  }

  {
    // The other side of that boundary: 40 entries fit exactly and must still be accepted.
    std::vector<char> buffer = MakeBlock({});
    HeaderAt(buffer, 0)->numItems = 40;
    ParseOutcome outcome = Parse(buffer);
    Check(outcome.result == 0, "item table of exactly 40 entries is accepted");
  }

  // --- Item extent bounds --------------------------------------------------------------------

  {
    std::vector<char> buffer = MakeBlock({{k_hmd2GenDataItemIdGazeCalib, 0x1F0, 0x40}});
    ParseOutcome outcome = Parse(buffer);
    Check(outcome.result == -1, "item extending past its block end is rejected");
  }

  {
    // offset + size wraps a uint32_t, which slipped through the old addition-based check.
    std::vector<char> buffer = MakeBlock({{k_hmd2GenDataItemIdGazeCalib, 0xFFFFFFF0u, 0x20}});
    ParseOutcome outcome = Parse(buffer);
    Check(outcome.result == -1, "item offset + size overflow is rejected");
  }

  {
    // The regression that motivated the fix: item offsets are block-relative, but the old check
    // compared them against the whole payload size. In the second block that was too permissive by
    // exactly one block, letting a read run off the end of the buffer.
    std::vector<char> buffer = MakeBlock({{k_hmd2GenDataItemIdGazeCalib, 0x100, 0x40}});
    std::vector<char> second = MakeBlock({{k_hmd2GenDataItemIdGazeCalib, 0x1F0, 0x20}});
    buffer.insert(buffer.end(), second.begin(), second.end());
    ParseOutcome outcome = Parse(buffer);
    Check(outcome.result == -1, "second block item is bounded by its own block, not the payload");
  }

  {
    // Same shape, but the item genuinely fits. This must still be accepted.
    std::vector<char> buffer = MakeBlock({{k_hmd2GenDataItemIdGazeCalib, 0x100, 0x40}});
    std::vector<char> second = MakeBlock({{k_hmd2GenDataItemIdGazeCalib, 0x1C0, 0x40}});
    buffer.insert(buffer.end(), second.begin(), second.end());
    ParseOutcome outcome = Parse(buffer);
    Check(outcome.result == 0 && outcome.calibItems == 2, "second block item ending exactly at the block end is accepted");
  }

  // --- Header validation ---------------------------------------------------------------------

  {
    std::vector<char> buffer = MakeBlock({});
    HeaderAt(buffer, 0)->magic[1] = 'X';
    Check(Parse(buffer).result == -1, "bad magic is rejected");
  }

  {
    std::vector<char> buffer = MakeBlock({});
    HeaderAt(buffer, 0)->minSize = 0x100;
    Check(Parse(buffer).result == -1, "bad minSize is rejected");
  }

  {
    std::vector<char> buffer = MakeBlock({});
    HeaderAt(buffer, 0)->size = k_blockSize * 2;
    Check(Parse(buffer).result == -1, "block claiming more bytes than remain is rejected");
  }

  {
    std::vector<char> buffer(0x100, 0);
    Check(Parse(buffer).result == -1, "payload shorter than one block granularity is rejected");
  }

  {
    // A trailing partial block after a valid one is an error, not a silent truncation.
    std::vector<char> buffer = MakeBlock({});
    buffer.resize(k_blockSize + 0x80, 0);
    Check(Parse(buffer).result == -1, "trailing partial block is rejected");
  }

  std::printf("\n%s (%d failure%s)\n", g_failures == 0 ? "PASSED" : "FAILED", g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
