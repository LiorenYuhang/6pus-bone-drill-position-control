#include "h30_imu/source_capture_writer.hpp"

#include <iomanip>
#include <utility>
#include <vector>

namespace h30_imu {

SourceCaptureWriter::~SourceCaptureWriter() {
  stop();
}

bool SourceCaptureWriter::start(const std::string & path, std::size_t queue_capacity) {
  stop();
  if (path.empty() || queue_capacity == 0u) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  path_ = path;
  queue_capacity_ = queue_capacity;
  queue_.clear();
  enqueued_rows_ = 0u;
  written_rows_ = 0u;
  overflow_drops_ = 0u;
  writer_errors_ = 0u;
  stopping_ = false;
  out_.open(path_, std::ios::out | std::ios::trunc);
  if (!out_.is_open()) {
    ++writer_errors_;
    return false;
  }
  enabled_ = true;
  write_header();
  writer_thread_ = std::thread(&SourceCaptureWriter::run, this);
  return true;
}

bool SourceCaptureWriter::enqueue(const SourceCaptureRow & row) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    return true;
  }
  if (queue_.size() >= queue_capacity_) {
    ++overflow_drops_;
    return false;
  }
  queue_.push_back(row);
  ++enqueued_rows_;
  cv_.notify_one();
  return true;
}

void SourceCaptureWriter::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_ && !writer_thread_.joinable()) {
      return;
    }
    stopping_ = true;
    cv_.notify_one();
  }
  if (writer_thread_.joinable()) {
    writer_thread_.join();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (out_.is_open()) {
    out_.flush();
    out_.close();
  }
  enabled_ = false;
  stopping_ = false;
}

SourceCaptureStats SourceCaptureWriter::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  SourceCaptureStats s;
  s.enabled = enabled_;
  s.enqueued_rows = enqueued_rows_;
  s.written_rows = written_rows_;
  s.overflow_drops = overflow_drops_;
  s.writer_errors = writer_errors_;
  s.queue_depth = queue_.size();
  s.queue_capacity = queue_capacity_;
  s.path = path_;
  return s;
}

void SourceCaptureWriter::run() {
  while (true) {
    std::vector<SourceCaptureRow> batch;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [&]() { return stopping_ || !queue_.empty(); });
      while (!queue_.empty()) {
        batch.push_back(queue_.front());
        queue_.pop_front();
      }
      if (batch.empty() && stopping_) {
        break;
      }
    }

    for (const auto & row : batch) {
      try {
        write_row(row);
      } catch (...) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++writer_errors_;
      }
    }

    bool done = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (out_.is_open()) {
        out_.flush();
      }
      done = stopping_ && queue_.empty();
    }
    if (done) {
      break;
    }
  }
}

void SourceCaptureWriter::write_header() {
  out_ << "publisher_sequence,tid,has_sample_timestamp,sample_timestamp_us,"
    "has_data_ready_timestamp,data_ready_timestamp_us,host_monotonic_arrival_time_ns,"
    "ros_header_stamp_ns,gx_rad_s,gy_rad_s,gz_rad_s,ax_m_s2,ay_m_s2,az_m_s2,"
    "qx,qy,qz,qw\n";
}

void SourceCaptureWriter::write_row(const SourceCaptureRow & row) {
  out_ << row.publisher_sequence << ','
    << row.tid << ','
    << (row.has_sample_timestamp ? 1 : 0) << ','
    << row.sample_timestamp_us << ','
    << (row.has_data_ready_timestamp ? 1 : 0) << ','
    << row.data_ready_timestamp_us << ','
    << row.host_monotonic_arrival_time_ns << ','
    << row.ros_header_stamp_ns << ','
    << std::setprecision(17)
    << row.angular_velocity_rad_s[0] << ','
    << row.angular_velocity_rad_s[1] << ','
    << row.angular_velocity_rad_s[2] << ','
    << row.linear_acceleration_m_s2[0] << ','
    << row.linear_acceleration_m_s2[1] << ','
    << row.linear_acceleration_m_s2[2] << ','
    << row.orientation_xyzw[0] << ','
    << row.orientation_xyzw[1] << ','
    << row.orientation_xyzw[2] << ','
    << row.orientation_xyzw[3] << '\n';
  if (!out_) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++writer_errors_;
  } else {
    std::lock_guard<std::mutex> lock(mutex_);
    ++written_rows_;
  }
}

}  // namespace h30_imu
