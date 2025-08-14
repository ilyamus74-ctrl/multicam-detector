#pragma once
#include <vector>
#include <unordered_map>
#include "common_types.h"

namespace mc {

struct FuserParams {
  int ttl_ms = 2000;          // объект «жив» столько после последнего совпадения
  float iou_gate = 0.2f;      // для межкамерного склеивания
  int time_gate_ms = 200;     // окно времени для совпадений
};

class GlobalIdFuser {
public:
  virtual ~GlobalIdFuser() = default;

  // Внести локальные треки от камеры
  virtual void Ingest(const CamId& cam, std::vector<Track>&& tracks, usec_t ts_us) = 0;

  // Получить актуальные глобальные объекты
  virtual std::vector<GlobalObject> Snapshot() const = 0;

  // Сбросить всё (при полной реконфигурации)
  virtual void Reset() = 0;
};

std::unique_ptr<GlobalIdFuser> CreateGlobalIdFuser(const FuserParams& p);

} // namespace mc
