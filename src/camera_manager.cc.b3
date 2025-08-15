
// src/camera_manager.cc
#include "camera_manager.h"
#include "v4l2_profiles.h"
#include "npu_worker_pool.h"
#include "tracker_sort.h"
#include "global_id_fuser.h"
#include "web_server.h"

#include <unordered_map>
#include <thread>
#include <mutex>
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

namespace mc {

class CameraManagerImpl : public CameraManager {
  AppConfig                               cfg_;
  std::unique_ptr<V4L2Profiles>           profiles_;
  std::unique_ptr<NPUWorkerPool>          npu_;
  std::unique_ptr<GlobalIdFuser>          fuser_;
  std::unique_ptr<WebServer>              web_;

  std::unordered_map<CamId, std::unique_ptr<CameraWorker>> cams_;
  std::unordered_map<CamId, std::unique_ptr<TrackerSORT>>  trackers_;

  OnLocalTracks                           on_local_;
  CameraManager::OnRenderedFrame          on_rendered_;     // NEW: колбэк кадра
  CamId                                   display_cam_;

  mutable std::mutex                      m_;               // FIX: mutex должен быть mutable

  // --- by-id → /dev/videoX по подстроке имени ---
  static std::string ResolveByIdContains(const std::optional<std::string>& needle) {
    if (!needle || needle->empty()) return {};
    const fs::path byid("/dev/v4l/by-id");
    if (!fs::exists(byid)) {
      std::cerr << "[cams] /dev/v4l/by-id not found\n";
      return {};
    }
    for (const auto& de : fs::directory_iterator(byid)) {
      const auto name = de.path().filename().string();
      if (name.find(*needle) != std::string::npos) {
        std::error_code ec;
        auto target = fs::read_symlink(de.path(), ec);
        if (ec) {
          std::cerr << "[cams] read_symlink failed for " << de.path()
                    << ": " << ec.message() << "\n";
          continue;
        }
        fs::path abs = (byid / target).lexically_normal();
        std::cout << "[cams] match by-id \"" << *needle << "\" -> " << abs << "\n";
        return abs.string();
      }
    }
    std::cerr << "[cams] no match for by_id_contains=\"" << *needle << "\"\n";
    return {};
  }

public:
  bool Start(const AppConfig& cfg) override {
    cfg_ = cfg;
    display_cam_ = cfg.http.display_camera;

    // Профили V4L2
    profiles_ = CreateV4L2Profiles(cfg.profiles.bright,
                                   cfg.profiles.indoor,
                                   cfg.profiles.dark,
                                   cfg.profiles.sw);

    // Веб-сервер
    web_ = CreateWebServer();
    web_->SetOnSwitchCamera([this](const CamId& id){
      std::scoped_lock lk(m_);
      display_cam_ = id;
      std::cout << "[http] switch display -> " << id << std::endl;
    });
    WebServerConfig wcfg{ cfg.http.port, cfg.http.http_fps_limit };
    if (!web_->Start(wcfg)) {
      std::cerr << "[http] start failed on port " << cfg.http.port << "\n";
      return false;
    }

    // Фьюзер глобальных ID — создаём до колбэков
    fuser_ = CreateGlobalIdFuser({});

    // Пул NPU
    npu_ = CreateNPUWorkerPool();
    npu_->SetOnDetections([this](const CamId& cam, usec_t ts, std::vector<Detection>&& dets){
      std::scoped_lock lk(m_);
      auto it = trackers_.find(cam);
      if (it == trackers_.end()) return;

      auto tracks = it->second->Update(dets, ts);
      if (on_local_) on_local_(cam, std::vector<Track>(tracks), ts);

      fuser_->Ingest(cam, std::move(tracks), ts);
      auto snap = fuser_->Snapshot();
      web_->PushMeta(snap, ts);
    });

    // (демо) один воркер/модель
    std::vector<NpuModelConfig> workers = {
      { "model_rknn/yolov8n_640x640_9out_fp32.rknn", 0, 640, 640 }
    };
    if (!npu_->Start(workers)) {
      std::cerr << "[npu] start failed\n";
      return false;
    }

    // Запуск камер
    for (const auto& c : cfg.cameras) {
      auto dev = ResolveByIdContains(c.match.by_id_contains);
      if (dev.empty()) {
        std::cerr << "[cams] skip \"" << c.id << "\" (device not found)\n";
        continue;
      }

      auto w = CreateCameraWorker();

      // сырые кадры → в NPU
      w->SetOnFrame([this](FramePacket&& f){
        npu_->Submit(std::move(f));
        // C2: здесь же прокинем в on_rendered_ / web_->PushRenderedFrame(...)
      });

      // детекции (если приходят напрямую от камеры)
      w->SetOnDetections([this](const CamId& cam, usec_t ts, std::vector<Detection>&& dets){
        std::scoped_lock lk(m_);
        auto it = trackers_.find(cam);
        if (it == trackers_.end()) return;

        auto tracks = it->second->Update(dets, ts);
        if (on_local_) on_local_(cam, std::vector<Track>(tracks), ts);

        fuser_->Ingest(cam, std::move(tracks), ts);
        auto snap = fuser_->Snapshot();
        web_->PushMeta(snap, ts);
      });

      CameraOpenParams p;
      p.cam_id        = c.id;
      p.dev_path      = dev;
      p.width         = c.w;
      p.height        = c.h;
      p.fps           = c.fps;
      p.pixfmt        = c.pixfmt;
      p.npu_worker    = c.npu_worker;
      p.auto_profiles = c.auto_profiles;

      if (!w->Start(p)) {
        std::cerr << "[cams] start failed: " << c.id << " @ " << dev << "\n";
        continue;
      }

      trackers_[c.id] = CreateTrackerSORT({});
      cams_[c.id] = std::move(w);

      std::cout << "[cams] started \"" << c.id << "\" @ " << dev
                << " (" << c.w << "x" << c.h << "@" << c.fps
                << ", " << c.pixfmt << ")\n";
    }

    return true;
  }

  void Stop() override {
    for (auto& [id, w] : cams_) w->Stop();
    cams_.clear();
    if (web_) web_->Stop();
    if (npu_) npu_->Stop();
  }

  void SetOnLocalTracks(OnLocalTracks cb) override {
    std::scoped_lock lk(m_);
    on_local_ = std::move(cb);
  }

  // NEW: реализация чисто виртуальной SetOnRenderedFrame
  void SetOnRenderedFrame(OnRenderedFrame cb) override {
    std::scoped_lock lk(m_);
    on_rendered_ = std::move(cb);
  }

  std::vector<std::pair<CamId, CamState>> List() const override {
    std::scoped_lock lk(m_);
    std::vector<std::pair<CamId, CamState>> v;
    v.reserve(cams_.size());
    for (const auto& kv : cams_) v.emplace_back(kv.first, kv.second->State());
    return v;
  }

  void SetDisplayCamera(const CamId& id) override {
    std::scoped_lock lk(m_);
    display_cam_ = id;
  }

  CamId GetDisplayCamera() const override {
    std::scoped_lock lk(m_);
    return display_cam_;
  }
};

std::unique_ptr<CameraManager> CreateCameraManager() {
  return std::make_unique<CameraManagerImpl>();
}

} // namespace mc
