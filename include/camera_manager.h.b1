#pragma once
#include <vector>
#include <memory>
#include <functional>
#include "common_types.h"
#include "camera_worker.h"
#include "v4l2_profiles.h"   // ← вот это нужно

namespace mc {

struct CameraMatch {
  std::optional<std::string> by_id_contains; // стаб. имя из /dev/v4l/by-id
};

struct CameraConfig {
  CamId id;
  CameraMatch match;
  int w=1280, h=720, fps=30;
  std::string pixfmt="MJPG";
  int npu_worker = 0;
  bool auto_profiles = true;
};

struct ProfileConfig {
  V4L2Profile bright, indoor, dark;
  ProfileSwitchSettings sw;
};

struct HttpConfig {
  int port = 8080;
  int http_fps_limit = 20;
  CamId display_camera = "";
};

struct AppConfig {
  HttpConfig http;
  ProfileConfig profiles;
  std::vector<CameraConfig> cameras;
  // npu workers и прочее можно добавить позже
};

class CameraManager {
public:
  using OnLocalTracks = std::function<void(const CamId&, std::vector<Track>&&, usec_t)>;

  virtual ~CameraManager() = default;

  virtual bool Start(const AppConfig& cfg) = 0;
  virtual void Stop() = 0;

  // Хотплаг событийно: менеджер сам создаёт/убивает CameraWorker
  // Внешний мир подписывается на локальные треки
  virtual void SetOnLocalTracks(OnLocalTracks cb) = 0;

  // Список активных камер/состояний
  virtual std::vector<std::pair<CamId, CamState>> List() const = 0;

  // Ручное переключение активной камеры для вывода
  virtual void SetDisplayCamera(const CamId& id) = 0;
  virtual CamId GetDisplayCamera() const = 0;
};

std::unique_ptr<CameraManager> CreateCameraManager();

bool LoadConfigFromFile(const std::string& path, AppConfig* out); // JSON → AppConfig

} // namespace mc
