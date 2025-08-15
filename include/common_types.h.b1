#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <optional>
#include <memory>

namespace mc {

// Идентификаторы
using CamId         = std::string;
using LocalTrackId  = int32_t;
using GlobalId      = int64_t;

// Время — монотонные микросекунды
using usec_t        = uint64_t;

// Базовые структуры
struct BBox {
  float x, y, w, h;   // в пикселях
  float score;        // 0..1
  int   cls;          // индекс класса
};

struct Detection {
  BBox box;
  usec_t ts_us;
};

struct Track {
  LocalTrackId track_id;     // стабильный в рамках одной камеры
  BBox box;
  usec_t ts_us;
  // опционально: скорость/направление после Калмана
  std::optional<float> vx, vy;   // px/s
};

struct GlobalObject {
  GlobalId id;
  CamId cam_id;               // где последний раз видели
  Track last_track;           // последняя локальная оценка
  usec_t last_seen_us;
};

// Пакет кадра → инференс
struct FramePacket {
  CamId cam_id;
  int frame_w, frame_h;       // размеры кадра
  usec_t ts_us;
  // Raw / RGB / NV12 — что используешь внутри (тут только forward-указатель)
  void* image_ptr = nullptr;  // ответственность владения на стороне захвата
};

} // namespace mc
