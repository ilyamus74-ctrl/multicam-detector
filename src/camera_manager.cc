
// src/camera_manager.cc

// ВКЛ/ВЫКЛ демо-боксы поверх MJPEG (без NPU)
#define MC_DEBUG_FAKE_META 1

#include "camera_manager.h"
#include "v4l2_profiles.h"
#include "npu_worker_pool.h"
#include "tracker_sort.h"
#include "global_id_fuser.h"
#include "web_server.h"

#include <unordered_map>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <filesystem>
#include <cmath>      // <-- для std::sin
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

namespace fs = std::filesystem;

static bool supports_mjpeg(const std::string& dev, int w=1280, int h=720) {
  int fd = open(dev.c_str(), O_RDWR | O_NONBLOCK);
  if (fd < 0) return false;
  v4l2_format fmt{};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width  = w;
  fmt.fmt.pix.height = h;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  bool ok = (ioctl(fd, VIDIOC_TRY_FMT, &fmt) == 0) && (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG);
  close(fd);
  return ok;
}

namespace mc {

class CameraManagerImpl : public CameraManager {
  AppConfig                               cfg_;
  std::unique_ptr<V4L2Profiles>           profiles_;
  std::unique_ptr<NPUWorkerPool>          npu_;
  std::unique_ptr<GlobalIdFuser>          fuser_;
  std::unique_ptr<WebServer>              web_;

  // workers / трекеры
  std::unordered_map<CamId, std::unique_ptr<CameraWorker>> cams_;
  std::unordered_map<CamId, std::unique_ptr<TrackerSORT>>  trackers_;
  std::unordered_map<CamId, std::string>                   cam_dev_;   // текущий /dev/... для камеры

  // колбэки наружу
  OnLocalTracks                           on_local_;
  OnRenderedFrame                         on_rendered_;

  // UI-состояние
  CamId                                   display_cam_;

  // синхронизация
  mutable std::mutex                      m_;

  // фоновый хот-плаг/перезапуск
  std::thread                             hotplug_th_;
  std::atomic<bool>                       hotplug_run_{false};

  // --- by-id → /dev/videoX по подстроке имени ---
  static std::string ResolveByIdContains(const std::optional<std::string>& needle) {
    const fs::path byid("/dev/v4l/by-id");
    if (needle && !needle->empty()) {
      if (!fs::exists(byid)) {
        std::cerr << "[cams] /dev/v4l/by-id not found\n";
      } else {
        for (const auto& de : fs::directory_iterator(byid)) {
          const auto name = de.path().filename().string();
          if (name.find(*needle) != std::string::npos) {
            std::error_code ec;
            auto target = fs::read_symlink(de.path(), ec);
            if (ec) {
              std::cerr << "[cams] read_symlink failed for " << de.path()
                        << ": " << ec.message() << "\n";
              break;
            }
            fs::path abs = (byid / target).lexically_normal();
            std::cout << "[cams] match by-id \"" << *needle << "\" -> " << abs << "\n";
            return abs.string();
          }
        }
        std::cerr << "[cams] no match for by_id_contains=\"" << *needle << "\"\n";
      }
    }
    // fallback: первый доступный /dev/video*
    for (int i = 0; i < 8; ++i) {
	fs::path p = fs::path("/dev") / ("video" + std::to_string(i));
	if (fs::exists(p) && supports_mjpeg(p.string())) {
	    std::cout << "[cams] fallback -> " << p << "\n";
	    return p.string();
	    }
    }

    return {};
  }

  // ---- запуск ОДНОЙ камеры (внутри под мьютексом) ----
  bool StartOneCameraLocked(const CameraConfig& c, const std::string& dev) {
    auto w = CreateCameraWorker();

    // кадр: MJPG → веб (и наружу), сырое → NPU
    w->SetOnFrame([this](FramePacket&& f){
      const CamId  cam = f.cam_id;
      const usec_t ts  = f.ts_us;

#ifdef MC_DEBUG_FAKE_META
      {
        // простейший «бегающий» бокс, чтобы видеть overlay
        double t = (f.ts_us % 5000000ULL) / 5e6; // 0..1 каждые 5 секунд
        float x = 50.f + 300.f * (float)t;
        float y = 50.f + 50.f  * (float)std::sin(t * 6.2831853); // 2π
        GlobalObject g{};
        g.id = 1;
        g.cam_id = f.cam_id;
        g.last_track.ts_us = f.ts_us;
        g.last_track.box = {x, y, 120.f, 80.f, 0.88f, 0};
        g.last_seen_us = f.ts_us;
        std::vector<GlobalObject> vec{g};
        if (web_) web_->PushMeta(vec, f.ts_us);
      }
#endif

      if (f.fmt == FrameFormat::MJPG && f.jpeg_data && f.jpeg_size > 0) {
        CamId display;
        {
          std::scoped_lock lk(m_);
          display = display_cam_;
        }
        if (web_ && (display.empty() || display == cam)) {
          web_->PushRenderedFrame(cam, f.jpeg_data, f.jpeg_size, ts);
        }
        if (on_rendered_) on_rendered_(cam, f.jpeg_data, f.jpeg_size, ts);
        // MJPG в NPU сейчас не отправляем — возвращаемся
        return;
      }

      // иначе — в NPU
      npu_->Submit(std::move(f));
    });

    // детекции напрямую (если минуем NPU)
    w->SetOnDetections([this](const CamId& cam, usec_t ts, std::vector<Detection>&& dets){
      std::scoped_lock lk(m_);
      auto it = trackers_.find(cam);
      if (it == trackers_.end()) return;

      auto tracks = it->second->Update(dets, ts);
      if (on_local_) on_local_(cam, tracks, ts);

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
      return false;
    }

    trackers_[c.id] = CreateTrackerSORT({});
    cams_[c.id] = std::move(w);
    cam_dev_[c.id] = dev;

    std::cout << "[cams] started \"" << c.id << "\" @ " << dev
              << " (" << c.w << "x" << c.h << "@" << c.fps
              << ", " << c.pixfmt << ")\n";
    return true;
  }

  void StopOneCameraLocked(const CamId& id) {
    auto it = cams_.find(id);
    if (it != cams_.end()) {
      it->second->Stop();
      cams_.erase(it);
    }
    trackers_.erase(id);
    cam_dev_.erase(id);
    std::cout << "[cams] stopped \"" << id << "\"\n";
  }

  // простой хот-плаг цикл: проверка статуса и перезапуск
  void StartHotplugLoop() {
    hotplug_run_ = true;
    hotplug_th_ = std::thread([this]{
      while (hotplug_run_) {
        for (const auto& c : cfg_.cameras) {
          bool need_restart = false;
          std::string dev_known;

          {
            std::scoped_lock lk(m_);
            auto itW = cams_.find(c.id);
            if (itW == cams_.end()) {
              need_restart = true; // ещё не поднята — пробуем
            } else {
              auto st = itW->second->State();
              if (st != CamState::RUN) {
                StopOneCameraLocked(c.id);
                need_restart = true;
              } else {
                auto itd = cam_dev_.find(c.id);
                if (itd != cam_dev_.end()) dev_known = itd->second;
                if (!dev_known.empty() && !fs::exists(dev_known)) {
                  StopOneCameraLocked(c.id);
                  need_restart = true;
                }
              }
            }
          }

          if (need_restart) {
            auto dev = ResolveByIdContains(c.match.by_id_contains);
            if (!dev.empty()) {
              std::scoped_lock lk(m_);
              StartOneCameraLocked(c, dev);
            }
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
      }
    });
  }

public:
  bool Start(const AppConfig& cfg) override {
    cfg_ = cfg;
    display_cam_ = cfg.http.display_camera;

    profiles_ = CreateV4L2Profiles(cfg.profiles.bright,
                                   cfg.profiles.indoor,
                                   cfg.profiles.dark,
                                   cfg.profiles.sw);

    // Web
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

    fuser_ = CreateGlobalIdFuser({});

    // NPU
    npu_ = CreateNPUWorkerPool();
    npu_->SetOnDetections([this](const CamId& cam, usec_t ts, std::vector<Detection>&& dets){
      std::scoped_lock lk(m_);
      auto it = trackers_.find(cam);
      if (it == trackers_.end()) return;

      auto tracks = it->second->Update(dets, ts);
      if (on_local_) on_local_(cam, tracks, ts);

      fuser_->Ingest(cam, std::move(tracks), ts);
      auto snap = fuser_->Snapshot();
      web_->PushMeta(snap, ts);
    });

    // demo model (заглушка)
    std::vector<NpuModelConfig> workers = {
      { "model_rknn/yolov8n_640x640_9out_fp32.rknn", 0, 640, 640 }
    };
    if (!npu_->Start(workers)) {
      std::cerr << "[npu] start failed\n";
      return false;
    }

    // попробуем стартануть камеры сразу (если уже подключены)
    for (const auto& c : cfg.cameras) {
      auto dev = ResolveByIdContains(c.match.by_id_contains);
      if (dev.empty()) {
        std::cerr << "[cams] skip \"" << c.id << "\" (device not found at start)\n";
        continue;
      }
      std::scoped_lock lk(m_);
      StartOneCameraLocked(c, dev);
    }

    // и запустим хот-плаг наблюдатель
    StartHotplugLoop();

    return true;
  }

  void Stop() override {
    hotplug_run_ = false;
    if (hotplug_th_.joinable()) hotplug_th_.join();

    {
      std::scoped_lock lk(m_);
      for (auto& [id, w] : cams_) w->Stop();
      cams_.clear();
      trackers_.clear();
      cam_dev_.clear();
    }
    if (web_) web_->Stop();
    if (npu_) npu_->Stop();
  }

  void SetOnLocalTracks(OnLocalTracks cb) override {
    std::scoped_lock lk(m_);
    on_local_ = std::move(cb);
  }

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
