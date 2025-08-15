// include/camera_manager.h
#pragma once
#include <vector>
#include <memory>
#include <functional>
#include <optional>
#include "common_types.h"
#include "camera_worker.h"
#include "v4l2_profiles.h"

namespace mc {

struct CameraMatch {
  std::optional<std::string> by_id_contains; // подстрока из /dev/v4l/by-id
};

struct CameraConfig {
  CamId        id;
  CameraMatch  match;
  int          w = 1280, h = 720, fps = 30;
  std::string  pixfmt = "MJPG";
  int          npu_worker = 0;
  bool         auto_profiles = true;
};

struct ProfileConfig {
  V4L2Profile          bright, indoor, dark;
  ProfileSwitchSettings sw;
};

struct HttpConfig {
  int    port = 8080;
  int    http_fps_limit = 20;
  CamId  display_camera = "";
};

struct AppConfig {
  HttpConfig                http;
  ProfileConfig             profiles;
  std::vector<CameraConfig> cameras;
};

// ==== callbacks start ====
// локальные треки (на камере)
class CameraManager {
public:
  using OnLocalTracks   = std::function<void(const CamId&, std::vector<Track>&, usec_t)>;

  // NEW: колбэк готового JPEG кадра (для веба)
  using OnRenderedFrame = std::function<void(const CamId&, const void*, size_t, usec_t)>;

  virtual ~CameraManager() = default;

  virtual bool Start(const AppConfig& cfg) = 0;
  virtual void Stop() = 0;

  virtual void SetOnLocalTracks(OnLocalTracks cb) = 0;

  // NEW: подписка на готовые кадры
  virtual void SetOnRenderedFrame(OnRenderedFrame cb) = 0;

  virtual std::vector<std::pair<CamId, CamState>> List() const = 0;

  virtual void SetDisplayCamera(const CamId& id) = 0;
  virtual CamId GetDisplayCamera() const = 0;
};
// ==== callbacks end ====

std::unique_ptr<CameraManager> CreateCameraManager();
bool LoadConfigFromFile(const std::string& path, AppConfig* out);

} // namespace mc
