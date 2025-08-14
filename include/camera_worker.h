#pragma once
#include <atomic>
#include <functional>
#include <optional>
#include "common_types.h"

namespace mc {

struct CameraOpenParams {
  CamId cam_id;
  std::string dev_path;     // /dev/v4l/by-id/...
  int width  = 1280;
  int height = 720;
  int fps    = 30;
  std::string pixfmt = "MJPG"; // желаемый
  int npu_worker = 0;          // куда слать инференс
  bool auto_profiles = true;
};

enum class CamState { INIT, RUN, ERROR, RETRY, STOPPED };

class CameraWorker {
public:
  using OnFrameCb = std::function<void(FramePacket&&)>;
  using OnDetectionsCb = std::function<void(const CamId&, usec_t, std::vector<Detection>&&)>;

  virtual ~CameraWorker() = default;

  virtual bool Start(const CameraOpenParams& p) = 0;
  virtual void Stop() = 0;

  virtual CamState State() const = 0;

  // Колбэки (ставит менеджер)
  virtual void SetOnFrame(OnFrameCb cb) = 0;           // в NPU
  virtual void SetOnDetections(OnDetectionsCb cb) = 0; // после постпроц/лок-трека

  // Служебное: ручное переключение профиля (для отладки)
  virtual void ForceProfile(const std::string& name) = 0;

  // Здоровье/метрики
  virtual int CurrentLuma() const = 0;
  virtual std::string CurrentProfile() const = 0;
};

std::unique_ptr<CameraWorker> CreateCameraWorker();

} // namespace mc
