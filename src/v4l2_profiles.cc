#include "v4l2_profiles.h"
#include <mutex>

namespace mc {
class V4L2ProfilesImpl : public V4L2Profiles {
  V4L2Profile bright_, indoor_, dark_;
  ProfileSwitchSettings sw_;
  const V4L2Profile* cur_ = nullptr;
  mutable std::mutex m_;
public:
  V4L2ProfilesImpl(const V4L2Profile& b, const V4L2Profile& i, const V4L2Profile& d, const ProfileSwitchSettings& sw)
  : bright_(b), indoor_(i), dark_(d), sw_(sw) { cur_ = &indoor_; }

  bool ApplyProfile(int /*fd*/, const V4L2Profile& p) override {
    // TODO: v4l2-ctl/ioctl. Сейчас только переключаем «текущий»
    std::scoped_lock lk(m_);
    cur_ = &p;
    return true;
  }

  int EstimateLuma(const FramePacket& /*frm*/) override {
    // TODO: оценка по кадру. Пока заглушка «средняя яркость»
    return 120;
  }

  const V4L2Profile& ChooseProfile(int luma) override {
    std::scoped_lock lk(m_);
    const int up = sw_.bright_if;
    const int dn = sw_.dark_if;
    const int h  = sw_.hysteresis;
    if (cur_==&indoor_) {
      if (luma > up+h) cur_=&bright_;
      else if (luma < dn-h) cur_=&dark_;
    } else if (cur_==&bright_) {
      if (luma < up-h) cur_=&indoor_;
      if (luma < dn-h) cur_=&dark_;
    } else { // dark_
      if (luma > dn+h) cur_=&indoor_;
      if (luma > up+h) cur_=&bright_;
    }
    return *cur_;
  }

  const V4L2Profile& Current() const override {
    std::scoped_lock lk(m_);
    return *cur_;
  }
};

std::unique_ptr<V4L2Profiles> CreateV4L2Profiles(
  const V4L2Profile& bright,
  const V4L2Profile& indoor,
  const V4L2Profile& dark,
  const ProfileSwitchSettings& sw) {
  return std::make_unique<V4L2ProfilesImpl>(bright, indoor, dark, sw);
}

} // namespace mc
