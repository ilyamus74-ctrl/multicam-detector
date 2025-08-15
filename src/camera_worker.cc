
#include "camera_worker.h"
#include "common_types.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/videodev2.h>

namespace mc {

namespace {
struct MMapBuf {
  void*  start = nullptr;
  size_t length = 0;
};
} // namespace

class CameraWorkerImpl : public CameraWorker {
public:
  CameraWorkerImpl() = default;
  ~CameraWorkerImpl() override { Stop(); }

  bool Start(const CameraOpenParams& p) override {
    if (running_) return true;
    params_ = p;

    // открыть девайс
    fd_ = ::open(params_.dev_path.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (fd_ < 0) {
      std::perror(("[cam] open " + params_.dev_path).c_str());
      state_ = CamState::ERROR;
      return false;
    }

    // запросить формат MJPEG
    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = params_.width;
    fmt.fmt.pix.height      = params_.height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG; // берём MJPEG
    fmt.fmt.pix.field       = V4L2_FIELD_ANY;
    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
      std::perror("[cam] VIDIOC_S_FMT (MJPEG)");
      ::close(fd_); fd_ = -1; state_ = CamState::ERROR;
      return false;
    }
    width_  = fmt.fmt.pix.width;
    height_ = fmt.fmt.pix.height;

    // fps
    {
      v4l2_streamparm parm{};
      parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      parm.parm.capture.timeperframe.numerator   = 1;
      parm.parm.capture.timeperframe.denominator = params_.fps > 0 ? params_.fps : 30;
      ioctl(fd_, VIDIOC_S_PARM, &parm);
      fps_ = parm.parm.capture.timeperframe.denominator > 0
           ? parm.parm.capture.timeperframe.denominator : params_.fps;
    }

    // выделить буферы
    v4l2_requestbuffers req{};
    req.count  = 3;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0 || req.count < 1) {
      std::perror("[cam] VIDIOC_REQBUFS");
      cleanup_fd();
      state_ = CamState::ERROR;
      return false;
    }

    bufs_.resize(req.count);
    for (unsigned i = 0; i < req.count; ++i) {
      v4l2_buffer b{};
      b.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      b.memory = V4L2_MEMORY_MMAP;
      b.index  = i;
      if (ioctl(fd_, VIDIOC_QUERYBUF, &b) < 0) {
        std::perror("[cam] VIDIOC_QUERYBUF");
        cleanup_all();
        state_ = CamState::ERROR;
        return false;
      }
      bufs_[i].length = b.length;
      bufs_[i].start  = mmap(nullptr, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, b.m.offset);
      if (bufs_[i].start == MAP_FAILED) {
        std::perror("[cam] mmap");
        cleanup_all();
        state_ = CamState::ERROR;
        return false;
      }
    }

    for (unsigned i = 0; i < req.count; ++i) {
      v4l2_buffer b{};
      b.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      b.memory = V4L2_MEMORY_MMAP;
      b.index  = i;
      if (ioctl(fd_, VIDIOC_QBUF, &b) < 0) {
        std::perror("[cam] VIDIOC_QBUF");
        cleanup_all();
        state_ = CamState::ERROR;
        return false;
      }
    }

    // stream on
    {
      v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        std::perror("[cam] VIDIOC_STREAMON");
        cleanup_all();
        state_ = CamState::ERROR;
        return false;
      }
    }

    running_ = true;
    state_   = CamState::RUN;
    thr_ = std::thread(&CameraWorkerImpl::captureLoop, this);
    return true;
  }

  void Stop() override {
    if (!running_) return;
    running_ = false;
    if (thr_.joinable()) thr_.join();
    cleanup_all();
    state_ = CamState::STOPPED;
  }

  CamState State() const override { return state_.load(); }

  void SetOnFrame(OnFrameCb cb) override { on_frame_ = std::move(cb); }
  void SetOnDetections(OnDetectionsCb cb) override { on_dets_ = std::move(cb); }

  void ForceProfile(const std::string& name) override {
    std::lock_guard<std::mutex> lk(prof_m_);
    cur_profile_ = name;
  }

  int CurrentLuma() const override {
    // Пока нет реальной оценки — вернём псевдо-значение.
    return 120;
  }

  std::string CurrentProfile() const override {
    std::lock_guard<std::mutex> lk(prof_m_);
    return cur_profile_;
  }

private:
  // cfg
  CameraOpenParams params_;

  // v4l2
  int fd_ = -1;
  std::vector<MMapBuf> bufs_;
  int width_ = 0, height_ = 0, fps_ = 0;

  // loop
  std::thread thr_{};
  std::atomic<bool> running_{false};
  std::atomic<CamState> state_{CamState::INIT};

  // callbacks
  OnFrameCb       on_frame_;
  OnDetectionsCb  on_dets_;

  // last JPEG storage (для безопасной передачи указателя в FramePacket)
  std::vector<uint8_t> last_jpeg_;
  mutable std::mutex prof_m_;
  std::string cur_profile_ = "indoor";

  void captureLoop() {
    using namespace std::chrono;

    while (running_) {
      // ждать готовый буфер
      fd_set fds; FD_ZERO(&fds); FD_SET(fd_, &fds);
      timeval tv{}; tv.tv_sec = 2; tv.tv_usec = 0;
      int r = ::select(fd_ + 1, &fds, nullptr, nullptr, &tv);
      if (r <= 0) {
        // timeout или ошибка — попробуем продолжить
        continue;
      }

      // снять из очереди
      v4l2_buffer b{};
      b.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      b.memory = V4L2_MEMORY_MMAP;
      if (ioctl(fd_, VIDIOC_DQBUF, &b) < 0) {
        // может быть EAGAIN — пропустим
        continue;
      }
      if (b.index >= bufs_.size()) {
        // вернуть, чтобы не потерять очередь
        ioctl(fd_, VIDIOC_QBUF, &b);
        continue;
      }

      // копия JPEG (без перекодирования)
      uint8_t* src = static_cast<uint8_t*>(bufs_[b.index].start);
      size_t   sz  = b.bytesused;
      last_jpeg_.assign(src, src + sz);

      // timestamp
      auto now_us = duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();

      // собрать пакет + вызвать колбэк
      if (on_frame_) {
        FramePacket f{};
        f.cam_id   = params_.cam_id;
        f.frame_w  = width_;
        f.frame_h  = height_;
        f.ts_us    = static_cast<usec_t>(now_us);
        f.fmt      = FrameFormat::MJPG;
        f.jpeg_data = last_jpeg_.data();
        f.jpeg_size = last_jpeg_.size();
        // image_ptr не используем для MJPEG
        on_frame_(std::move(f));
      }

      // вернуть буфер драйверу
      ioctl(fd_, VIDIOC_QBUF, &b);

      // лёгкая пауза чтобы не молотить 100% CPU, если FPS высокий
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  void cleanup_fd() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
  }

  void cleanup_all() {
    if (fd_ >= 0) {
      v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      ioctl(fd_, VIDIOC_STREAMOFF, &type);
    }
    for (auto& m : bufs_) {
      if (m.start && m.start != MAP_FAILED && m.length) {
        munmap(m.start, m.length);
      }
    }
    bufs_.clear();
    cleanup_fd();
  }
};

std::unique_ptr<CameraWorker> CreateCameraWorker() {
  return std::make_unique<CameraWorkerImpl>();
}

} // namespace mc
