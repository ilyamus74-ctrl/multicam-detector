#pragma once
#include <string>
#include <optional>
#include <vector>
#include "common_types.h"

namespace mc {

struct V4L2Profile {
  std::string name;     // "bright" / "indoor" / "dark"
  int exposure_abs;     // UVC exposure_time_absolute, ед. 1/10000 c (16ms = 160)
  int gain;             // аналоговый/цифровой gain, зависит от камеры
  int gamma;            // 80..200 (типично)
  int saturation;       // 0..255 (0 = ч/б)
  bool force_grey = false; // попытаться GREY pixfmt (если поддерживается)
};

struct ProfileSwitchSettings {
  std::string metric;   // "mean_luma"
  int bright_if;        // > 180
  int dark_if;          // < 60
  int hysteresis;       // 10
};

class V4L2Profiles {
public:
  virtual ~V4L2Profiles() = default;

  // Применить профиль к открытому FD камеры
  virtual bool ApplyProfile(int v4l2_fd, const V4L2Profile& p) = 0;

  // Оценить метрику (яркость/шум) по кадру (используешь свой путь данных)
  virtual int  EstimateLuma(const FramePacket& frm) = 0;

  // Выбрать профиль на основе метрики + гистерезиса
  virtual const V4L2Profile& ChooseProfile(int luma) = 0;

  // Текущий профиль
  virtual const V4L2Profile& Current() const = 0;
};

std::unique_ptr<V4L2Profiles> CreateV4L2Profiles(
  const V4L2Profile& bright,
  const V4L2Profile& indoor,
  const V4L2Profile& dark,
  const ProfileSwitchSettings& sw);

} // namespace mc
