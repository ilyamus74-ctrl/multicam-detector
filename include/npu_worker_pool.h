#pragma once
#include <vector>
#include <memory>
#include <functional>
#include "common_types.h"

namespace mc {

struct NpuModelConfig {
  std::string rknn_path;   // путь к .rknn
  int core_index = 0;      // 0/1/2
  int input_w = 640, input_h = 640;
};

class NPUWorkerPool {
public:
  using OnDetections = std::function<void(const CamId&, usec_t, std::vector<Detection>&&)>;

  virtual ~NPUWorkerPool() = default;

  // Инициализация нескольких воркеров (по ядрам)
  virtual bool Start(const std::vector<NpuModelConfig>& workers) = 0;
  virtual void Stop() = 0;

  // Отправить кадр на подходящий воркер (по cam_id → prebound core)
  virtual void Submit(FramePacket&& frame) = 0;

  // Колбэк с детекциями
  virtual void SetOnDetections(OnDetections cb) = 0;
};

std::unique_ptr<NPUWorkerPool> CreateNPUWorkerPool();

} // namespace mc
