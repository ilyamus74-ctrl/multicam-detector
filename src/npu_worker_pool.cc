#include "npu_worker_pool.h"
#include <mutex>
#include <thread>
#include <queue>

namespace mc {
class NPUWorkerPoolImpl : public NPUWorkerPool {
  std::vector<NpuModelConfig> cfg_;
  OnDetections cb_;
  std::mutex m_;
public:
  bool Start(const std::vector<NpuModelConfig>& workers) override {
    cfg_ = workers;
    return true;
  }
  void Stop() override {}

  void Submit(FramePacket&& f) override {
    // TODO: реальный RKNN. Сейчас шлём пустые детекции.
    std::scoped_lock lk(m_);
    if (cb_) cb_(f.cam_id, f.ts_us, {});
  }

  void SetOnDetections(OnDetections cb) override {
    std::scoped_lock lk(m_);
    cb_ = std::move(cb);
  }
};

std::unique_ptr<NPUWorkerPool> CreateNPUWorkerPool() {
  return std::make_unique<NPUWorkerPoolImpl>();
}
} // namespace mc
