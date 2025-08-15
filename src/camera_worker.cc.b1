#include "camera_worker.h"
#include <thread>
#include <atomic>
#include <chrono>
#include <iostream>

namespace mc {
class CameraWorkerImpl : public CameraWorker {
  CameraOpenParams p_;
  std::thread th_;
  std::atomic<bool> run_{false};
  std::atomic<CamState> st_{CamState::INIT};
  OnFrameCb on_frame_;
  OnDetectionsCb on_dets_;
  int luma_ = 120;
  std::string prof_ = "indoor";
public:
  ~CameraWorkerImpl(){ Stop(); }

  bool Start(const CameraOpenParams& p) override {
    p_ = p;
    run_ = true;
    st_ = CamState::RUN;
    th_ = std::thread([this]{
      using namespace std::chrono;
      auto period = milliseconds(1000 / std::max(1,p_.fps));
      while (run_) {
        auto now = duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
        FramePacket f; f.cam_id=p_.cam_id; f.frame_w=p_.width; f.frame_h=p_.height; f.ts_us=(usec_t)now; f.image_ptr=nullptr;
        if (on_frame_) on_frame_(std::move(f));
        // для наглядности шлём 1 пустую «детекцию» раз в 10 кадров — имитировать движение
        static int k=0; if (++k%10==0 && on_dets_) {
          Detection d{}; d.box={ (float)(k%p_.width), (float)(k%p_.height/2), 60, 60, 0.9f, 0 }; d.ts_us=(usec_t)now;
          std::vector<Detection> v{d};
          on_dets_(p_.cam_id, (usec_t)now, std::move(v));
        }
        std::this_thread::sleep_for(period);
      }
      st_ = CamState::STOPPED;
    });
    return true;
  }
  void Stop() override {
    if (!run_) return;
    run_ = false;
    if (th_.joinable()) th_.join();
  }

  CamState State() const override { return st_.load(); }

  void SetOnFrame(OnFrameCb cb) override { on_frame_ = std::move(cb); }
  void SetOnDetections(OnDetectionsCb cb) override { on_dets_ = std::move(cb); }

  void ForceProfile(const std::string& name) override { prof_=name; }
  int CurrentLuma() const override { return luma_; }
  std::string CurrentProfile() const override { return prof_; }
};

std::unique_ptr<CameraWorker> CreateCameraWorker() {
  return std::make_unique<CameraWorkerImpl>();
}
} // namespace mc
