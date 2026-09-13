#pragma once

// Session recorder for the gaze stream (EYE_TRACKING_PLAN.md 0.3).
//
// Phases 2, 4 and 5 all change the numbers the tracker produces, so there has to be a way to compare
// a change against the same input. This dumps every field of hmd2_gaze_status_t to CSV, plus a host
// receive timestamp, and optionally the raw IR eye camera frames.
//
// Polling runs on its own thread rather than off the render loop: the gaze stream can outrun vsync,
// and a dropped sample would show up as a frame_counter gap in the capture.

#include "hmd2_gaze.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <system_error>

namespace gaze_capture {

inline int64_t NowMicros() {
  using namespace std::chrono;
  static const steady_clock::time_point start = steady_clock::now();
  return duration_cast<microseconds>(steady_clock::now() - start).count();
}

// CSV emission. the header and the row are produced by walking the same description, so the two cannot drift apart as fields are added. Field names are only materialised by the header emitter.

struct CsvHeaderEmitter {
  std::string out;

  void Add(const char *prefix, const char *name) {
    if (!out.empty()) {
      out += ',';
    }
    out += prefix;
    out += name;
  }

  void operator()(const char *prefix, const char *name, double) { Add(prefix, name); }
  void operator()(const char *prefix, const char *name, int64_t) { Add(prefix, name); }
};

struct CsvRowEmitter {
  std::string out;

  void Add(const char *text) {
    if (!out.empty()) {
      out += ',';
    }
    out += text;
  }

  void operator()(const char *, const char *, double value) {
    char buffer[40];
    std::snprintf(buffer, sizeof(buffer), "%.9g", value);
    Add(buffer);
  }

  void operator()(const char *, const char *, int64_t value) {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%lld", static_cast<long long>(value));
    Add(buffer);
  }
};

template <typename Emit> void DescribeEye(const char *prefix, const hmd2_gaze_wearable_eye_t &eye, Emit &emit) {
  emit(prefix, "gaze_origin_valid", static_cast<double>(eye.is_gaze_origin_valid));
  emit(prefix, "gaze_origin_x", static_cast<double>(eye.gaze_origin_mm.x));
  emit(prefix, "gaze_origin_y", static_cast<double>(eye.gaze_origin_mm.y));
  emit(prefix, "gaze_origin_z", static_cast<double>(eye.gaze_origin_mm.z));
  emit(prefix, "gaze_dir_valid", static_cast<double>(eye.is_gaze_dir_valid));
  emit(prefix, "gaze_dir_x", static_cast<double>(eye.gaze_dir_norm.x));
  emit(prefix, "gaze_dir_y", static_cast<double>(eye.gaze_dir_norm.y));
  emit(prefix, "gaze_dir_z", static_cast<double>(eye.gaze_dir_norm.z));
  emit(prefix, "pupil_dia_valid", static_cast<double>(eye.is_pupil_dia_valid));
  emit(prefix, "pupil_dia_mm", static_cast<double>(eye.pupil_dia_mm));
  emit(prefix, "pupil_pos_valid", static_cast<double>(eye.is_pupil_pos_in_sensor_area_valid));
  emit(prefix, "pupil_pos_x", static_cast<double>(eye.pupil_pos_in_sensor_area.x));
  emit(prefix, "pupil_pos_y", static_cast<double>(eye.pupil_pos_in_sensor_area.y));
  emit(prefix, "pos_guide_valid", static_cast<double>(eye.is_pos_guide_valid));
  emit(prefix, "pos_guide_x", static_cast<double>(eye.pos_guide.x));
  emit(prefix, "pos_guide_y", static_cast<double>(eye.pos_guide.y));
  emit(prefix, "blink_valid", static_cast<double>(eye.is_blink_valid));
  emit(prefix, "blink", static_cast<double>(eye.blink));
}

template <typename Emit> void DescribeGazeStatus(const hmd2_gaze_status_t &s, int64_t hostRecvUs, Emit &emit) {
  emit("", "host_recv_us", static_cast<int64_t>(hostRecvUs));

  emit("", "magic0", static_cast<double>(s.magic[0]));
  emit("", "magic1", static_cast<double>(s.magic[1]));
  emit("", "version", static_cast<double>(s.version));
  emit("", "size", static_cast<double>(s.size));

  emit("", "exp_l", static_cast<double>(s.exp_l));
  emit("", "exp_r", static_cast<double>(s.exp_r));
  emit("", "led_status", static_cast<double>(s.led_status));
  emit("", "exp_counter_l", static_cast<double>(s.exp_counter_l));
  emit("", "exp_counter_r", static_cast<double>(s.exp_counter_r));
  emit("", "led_counter", static_cast<double>(s.led_counter));
  emit("", "dsp_return_code", static_cast<double>(s.dsp_return_code));

  emit("", "lens_left_x", static_cast<double>(s.lens_config.left.x));
  emit("", "lens_left_y", static_cast<double>(s.lens_config.left.y));
  emit("", "lens_left_z", static_cast<double>(s.lens_config.left.z));
  emit("", "lens_right_x", static_cast<double>(s.lens_config.right.x));
  emit("", "lens_right_y", static_cast<double>(s.lens_config.right.y));
  emit("", "lens_right_z", static_cast<double>(s.lens_config.right.z));

  emit("", "user_calibration_id", static_cast<double>(s.user_calibration_id));
  emit("", "fr_gaze_origin_x", static_cast<double>(s.fr_gaze_origin.x));
  emit("", "fr_gaze_origin_y", static_cast<double>(s.fr_gaze_origin.y));
  emit("", "fr_gaze_origin_z", static_cast<double>(s.fr_gaze_origin.z));
  emit("", "enabled_eye", static_cast<double>(s.enabled_eye));
  emit("", "motor_sequence", static_cast<double>(s.motor_sequence));
  emit("", "motor_strength", static_cast<double>(s.motor_strength));

  emit("", "wearable_timestamp", static_cast<int64_t>(s.wearable.timestamp));
  emit("", "wearable_frame_counter", static_cast<double>(s.wearable.frame_counter));

  DescribeEye("left_", s.wearable.left, emit);
  DescribeEye("right_", s.wearable.right, emit);

  emit("", "combined_origin_valid", static_cast<double>(s.wearable.is_gaze_origin_combined_valid));
  emit("", "combined_origin_x", static_cast<double>(s.wearable.gaze_origin_combined_mm.x));
  emit("", "combined_origin_y", static_cast<double>(s.wearable.gaze_origin_combined_mm.y));
  emit("", "combined_origin_z", static_cast<double>(s.wearable.gaze_origin_combined_mm.z));
  emit("", "combined_dir_valid", static_cast<double>(s.wearable.is_gaze_dir_combined_valid));
  emit("", "combined_dir_x", static_cast<double>(s.wearable.gaze_dir_combined_norm.x));
  emit("", "combined_dir_y", static_cast<double>(s.wearable.gaze_dir_combined_norm.y));
  emit("", "combined_dir_z", static_cast<double>(s.wearable.gaze_dir_combined_norm.z));
  emit("", "convergence_valid", static_cast<double>(s.wearable.is_convergence_distance_valid));

  emit("", "foveated_timestamp", static_cast<int64_t>(s.foveated.timestamp));
  emit("", "foveated_frame_counter", static_cast<double>(s.foveated.frame_counter));
  emit("", "foveated_tracking_state", static_cast<double>(s.foveated.tracking_state));
  emit("", "foveated_dir_left_x", static_cast<double>(s.foveated.gaze_dir_left_norm.x));
  emit("", "foveated_dir_left_y", static_cast<double>(s.foveated.gaze_dir_left_norm.y));
  emit("", "foveated_dir_left_z", static_cast<double>(s.foveated.gaze_dir_left_norm.z));
  emit("", "foveated_dir_right_x", static_cast<double>(s.foveated.gaze_dir_right_norm.x));
  emit("", "foveated_dir_right_y", static_cast<double>(s.foveated.gaze_dir_right_norm.y));
  emit("", "foveated_dir_right_z", static_cast<double>(s.foveated.gaze_dir_right_norm.z));
  emit("", "foveated_dir_combined_x", static_cast<double>(s.foveated.gaze_dir_combined_norm.x));
  emit("", "foveated_dir_combined_y", static_cast<double>(s.foveated.gaze_dir_combined_norm.y));
  emit("", "foveated_dir_combined_z", static_cast<double>(s.foveated.gaze_dir_combined_norm.z));
  emit("", "foveated_convergence_distance_mm", static_cast<double>(s.foveated.convergence_distance_mm));
}

// Capture destination

inline std::string MakeSessionDirectory() {
  const std::time_t now = std::time(nullptr);
  std::tm tm = {};
#ifdef _WIN32
  localtime_s(&tm, &now);
#else
  localtime_r(&now, &tm);
#endif

  char stamp[32];
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm);

  std::filesystem::path dir = std::filesystem::path("psvr2tk_captures") / stamp;
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  return dir.string();
}

// Status recorder 

class GazeRecorder {
public:
  bool IsActive() const { return m_active.load(); }
  uint64_t Rows() const { return m_rows.load(); }
  uint64_t Gaps() const { return m_gaps.load(); }

  std::string Path() {
    std::scoped_lock<std::mutex> lock(m_mutex);
    return m_path;
  }

  bool Start() {
    std::scoped_lock<std::mutex> lock(m_mutex);
    if (m_active.load()) {
      return true;
    }

    const std::string dir = MakeSessionDirectory();
    m_path = (std::filesystem::path(dir) / "gaze_status.csv").string();

    m_csv.open(m_path, std::ios::out | std::ios::trunc);
    if (!m_csv.is_open()) {
      m_path.clear();
      return false;
    }

    hmd2_gaze_status_t blank = {};
    CsvHeaderEmitter header;
    DescribeGazeStatus(blank, 0, header);
    m_csv << header.out << "\n";

    m_rows.store(0);
    m_gaps.store(0);
    m_haveLastFrame = false;
    m_active.store(true);
    return true;
  }

  void Stop() {
    std::scoped_lock<std::mutex> lock(m_mutex);
    if (!m_active.load()) {
      return;
    }
    m_active.store(false);
    m_csv.flush();
    m_csv.close();
  }

  void Write(const hmd2_gaze_status_t &status, int64_t hostRecvUs) {
    std::scoped_lock<std::mutex> lock(m_mutex);
    if (!m_active.load() || !m_csv.is_open()) {
      return;
    }

    // A capture with gaps is not usable as a reference, so surface them rather than silently recording a stream that skipped frames.
    if (m_haveLastFrame) {
      const uint32_t expected = m_lastFrame + 1;
      if (status.wearable.frame_counter != expected) {
        m_gaps.fetch_add(1);
      }
    }
    m_lastFrame = status.wearable.frame_counter;
    m_haveLastFrame = true;

    CsvRowEmitter row;
    DescribeGazeStatus(status, hostRecvUs, row);
    m_csv << row.out << "\n";

    m_rows.fetch_add(1);
  }

private:
  std::mutex m_mutex;
  std::ofstream m_csv;
  std::string m_path;
  std::atomic<bool> m_active{false};
  std::atomic<uint64_t> m_rows{0};
  std::atomic<uint64_t> m_gaps{0};
  bool m_haveLastFrame = false;
  uint32_t m_lastFrame = 0;
};

// IR eye image dumper 
class GazeImageDumper {
public:
  // Full slot size: 0x100 header + 0x200000 payload.
  static constexpr size_t k_frameSize = 0x200100;

  bool IsActive() const { return m_active; }
  int Written() const { return m_written; }
  int &Stride() { return m_stride; }
  int &MaxFrames() { return m_maxFrames; }
  const std::string &Directory() const { return m_dir; }

  bool Start() {
    if (m_active) {
      return true;
    }
    m_dir = MakeSessionDirectory();
    m_seen = 0;
    m_written = 0;
    m_active = true;
    return true;
  }

  void Stop() { m_active = false; }

  // Called from the render loop with a frame already copied out of shared memory by psvr2_toolkit_gaze_image_copy, so what lands on disk cannot tear.
  void MaybeDump(const unsigned char *image, uint32_t size) {
    if (!m_active || image == nullptr || size == 0) {
      return;
    }

    if (m_written >= m_maxFrames) {
      m_active = false;
      return;
    }

    if (m_stride > 1 && (m_seen++ % m_stride) != 0) {
      return;
    }

    char name[64];
    std::snprintf(name, sizeof(name), "eye_%05d.bin", m_written);

    std::ofstream out(std::filesystem::path(m_dir) / name, std::ios::binary);
    if (!out.is_open()) {
      m_active = false;
      return;
    }

    out.write(reinterpret_cast<const char *>(image), static_cast<std::streamsize>(size));
    ++m_written;
  }

private:
  bool m_active = false;
  int m_stride = 10;
  int m_maxFrames = 200;
  int m_seen = 0;
  int m_written = 0;
  std::string m_dir;
};

} // namespace gaze_capture
