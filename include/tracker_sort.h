#pragma once
#include <vector>
#include "common_types.h"

namespace mc {

struct SortParams {
  float iou_thresh = 0.3f;
  int max_age = 15;       // кадров без детекции
  int min_hits = 3;       // чтобы закрепить ID
  float process_noise = 1.0f;  // для Калмана
  float measurement_noise = 1.0f;
};

class TrackerSORT {
public:
  virtual ~TrackerSORT() = default;

  // Обновить трекер детекциями кадра; вернуть список активных треков (с упреждением)
  virtual std::vector<Track> Update(const std::vector<Detection>& dets, usec_t ts_us) = 0;

  // Текущие живые треки (без апдейта)
  virtual std::vector<Track> LiveTracks() const = 0;
};

std::unique_ptr<TrackerSORT> CreateTrackerSORT(const SortParams& p);

} // namespace mc
