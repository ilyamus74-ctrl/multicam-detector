
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
  std::unique_ptr<V4L2Profiles>   profiles_;
  std::unique_ptr<NPUWorkerPool>  npu_;
  std::unique_ptr<GlobalIdFuser>  fuser_;
  std::unique_ptr<WebServer>      web_;
  std::unordered_map<CamId, std::unique_ptr<CameraWorker>> cams_;
  std::unordered_map<CamId, std::unique_ptr<TrackerSORT>>  trackers_;
  OnLocalTracks   on_local_;
  OnRenderedFrame on_frame_;      // <--- ДОБАВЛЕНО
  CamId           display_cam_;
  std::mutex      m_;
public:
  bool Start(const AppConfig& cfg) override {
    cfg_ = cfg;
    display_cam_ = cfg.http.display_camera;

    profiles_ = CreateV4L2Profiles(cfg.profiles.bright, cfg.profiles.indoor, cfg.profiles.dark, cfg.profiles.sw);

    // Глобальный фьюзер создаём ДО установки коллбэков, чтобы не поймать гонку
    fuser_ = CreateGlobalIdFuser({});

    // веб
    web_ = CreateWebServer();
    web_->SetOnSwitchCamera([this](const CamId& id){
      std::scoped_lock lk(m_);
      display_cam_ = id;
      std::cout << "switch display -> " << id << std::endl;
    });
    web_->Start({cfg.http.port, cfg.http.http_fps_limit});

    // NPU: одна модель на демо
    npu_ = CreateNPUWorkerPool();
    npu_->SetOnDetections([this](const CamId& cam, usec_t ts, std::vector<Detection>&& dets){
      std::scoped_lock lk(m_);
      auto it = trackers_.find(cam);
      if (it==trackers_.end()) return;

      auto tracks = it->second->Update(dets, ts);
      if (on_local_) on_local_(cam, std::vector<Track>(tracks), ts);

      fuser_->Ingest(cam, std::move(tracks), ts);
      auto snap = fuser_->Snapshot();
      web_->PushMeta(snap, ts);
    });

    std::vector<NpuModelConfig> workers = { {"model_rknn/yolov8n_640x640_9out_fp32.rknn",0,640,640} };
    npu_->Start(workers);

    // камеры
    for (auto& c : cfg.cameras){
      auto w = CreateCameraWorker();

      // Из камеры приходит FramePacket: отправляем в NPU и (если задан) jpeg → веб
      w->SetOnFrame([this, camid=c.id](FramePacket&& f){
        // Пытаемся отдать сырое MJPG в веб (если подписчик есть)
        if (on_frame_) {
          // --- ВАЖНО ---
          // Ниже я предполагаю, что в FramePacket есть либо std::vector<uint8_t> mjpg,
          // либо поля mjpg_data/mjpg_size, а также метка времени ts_us.
          // Если имена у вас другие — покажи common_types.h, быстро поправлю.

          const void* jpg = nullptr;
          size_t      sz  = 0;

          // ВАРИАНТ A: std::vector<uint8_t> mjpg;
          // try/catch не нужен — просто раскомментируй если это твой вариант
          // if (!f.mjpg.empty()) { jpg = f.mjpg.data(); sz = f.mjpg.size(); }

          // ВАРИАНТ B: uint8_t* mjpg_data; size_t mjpg_size;
          // if (f.mjpg_data && f.mjpg_size) { jpg = f.mjpg_data; sz = f.mjpg_size; }

          // Если один из вариантов выше актуален — снимай комментарий.
          // Пока оставим «пусто», чтобы не звать веб без данных.

          if (jpg && sz) {
            on_frame_(camid, jpg, sz, f.ts_us);
          }
        }

        // Отправляем кадр дальше в NPU-пул
        npu_->Submit(std::move(f));
      });

      w->SetOnDetections([this](const CamId& cam, usec_t ts, std::vector<Detection>&& dets){
        auto it = trackers_.find(cam);
        if (it==trackers_.end()) return;

        auto tracks = it->second->Update(dets, ts);
        if (on_local_) on_local_(cam, std::vector<Track>(tracks), ts);

        fuser_->Ingest(cam, std::move(tracks), ts);
        auto snap = fuser_->Snapshot();
        web_->PushMeta(snap, ts);
      });

      // TODO: dev_path сейчас заглушка — позже подставим реальный by-id
      w->Start({c.id, "/dev/fake", c.w, c.h, c.fps, c.pixfmt, c.npu_worker, c.auto_profiles});
      trackers_[c.id] = CreateTrackerSORT({});
      cams_[c.id] = std::move(w);
    }

    return true;
  }

  void Stop() override {
    for (auto& [id,w] : cams_) w->Stop();
    cams_.clear();
    if (web_) web_->Stop();
    if (npu_) npu_->Stop();
  }

  void SetOnLocalTracks(OnLocalTracks cb) override { on_local_ = std::move(cb); }
  void SetOnRenderedFrame(OnRenderedFrame cb) override { on_frame_ = std::move(cb); }

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
