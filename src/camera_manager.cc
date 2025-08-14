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

namespace mc {

class CameraManagerImpl : public CameraManager {
  AppConfig cfg_;
  std::unique_ptr<V4L2Profiles> profiles_;
  std::unique_ptr<NPUWorkerPool> npu_;
  std::unique_ptr<GlobalIdFuser> fuser_;
  std::unique_ptr<WebServer> web_;
  std::unordered_map<CamId, std::unique_ptr<CameraWorker>> cams_;
  std::unordered_map<CamId, std::unique_ptr<TrackerSORT>> trackers_;
  OnLocalTracks on_local_;
  CamId display_cam_;
  std::mutex m_;
public:
  bool Start(const AppConfig& cfg) override {
    cfg_ = cfg;
    display_cam_ = cfg.http.display_camera;

    profiles_ = CreateV4L2Profiles(cfg.profiles.bright, cfg.profiles.indoor, cfg.profiles.dark, cfg.profiles.sw);
    npu_ = CreateNPUWorkerPool();
    npu_->SetOnDetections([this](const CamId& cam, usec_t ts, std::vector<Detection>&& dets){
      // лок-трекинг per-camera
      std::scoped_lock lk(m_);
      auto it = trackers_.find(cam);
      if (it==trackers_.end()) return;
      auto tracks = it->second->Update(dets, ts);
      if (on_local_) on_local_(cam, std::vector<Track>(tracks), ts);
      // фьюз в глобальные ID
      fuser_->Ingest(cam, std::move(tracks), ts);
      // пуш мета в веб
      auto snap = fuser_->Snapshot();
      web_->PushMeta(snap, ts);
    });

    // запустить веб
    web_ = CreateWebServer();
    web_->SetOnSwitchCamera([this](const CamId& id){
      std::scoped_lock lk(m_);
      display_cam_ = id;
      std::cout << "switch display -> " << id << std::endl;
    });
    web_->Start({cfg.http.port, cfg.http.http_fps_limit});

    // npu start (одна модель на 1 воркер — для демо)
    std::vector<NpuModelConfig> workers = { {"model_rknn/yolov8n_640x640_9out_fp32.rknn",0,640,640} };
    npu_->Start(workers);

    // создать камеры
    for (auto& c : cfg.cameras){
      auto w = CreateCameraWorker();
      w->SetOnFrame([this](FramePacket&& f){ npu_->Submit(std::move(f)); });
      w->SetOnDetections([this](const CamId& cam, usec_t ts, std::vector<Detection>&& dets){
        // путь детекций напрямую от камеры (если минуя NPU пул) — используем тот же код:
        auto it = trackers_.find(cam);
        if (it==trackers_.end()) return;
        auto tracks = it->second->Update(dets, ts);
        if (on_local_) on_local_(cam, std::vector<Track>(tracks), ts);
        fuser_->Ingest(cam, std::move(tracks), ts);
        auto snap = fuser_->Snapshot();
        web_->PushMeta(snap, ts);
      });
      w->Start({c.id, "/dev/fake", c.w, c.h, c.fps, c.pixfmt, c.npu_worker, c.auto_profiles});
      trackers_[c.id] = CreateTrackerSORT({});
      cams_[c.id] = std::move(w);
    }

    fuser_ = CreateGlobalIdFuser({});

    return true;
  }

  void Stop() override {
    for (auto& [id,w] : cams_) w->Stop();
    cams_.clear();
    if (web_) web_->Stop();
    if (npu_) npu_->Stop();
  }

  void SetOnLocalTracks(OnLocalTracks cb) override { on_local_ = std::move(cb); }

  std::vector<std::pair<CamId, CamState>> List() const override {
    std::vector<std::pair<CamId, CamState>> v;
    for (auto& kv : cams_) v.push_back({kv.first, kv.second->State()});
    return v;
  }

  void SetDisplayCamera(const CamId& id) override { display_cam_ = id; }
  CamId GetDisplayCamera() const override { return display_cam_; }
};

std::unique_ptr<CameraManager> CreateCameraManager(){
  return std::make_unique<CameraManagerImpl>();
}

} // namespace mc
