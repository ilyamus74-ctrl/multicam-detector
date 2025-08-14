#include "global_id_fuser.h"
#include <unordered_map>
#include <deque>
#include <atomic>
#include <algorithm>

namespace mc {
static float IoU(const BBox& a, const BBox& b) {
  float x1 = std::max(a.x, b.x);
  float y1 = std::max(a.y, b.y);
  float x2 = std::min(a.x+a.w, b.x+b.w);
  float y2 = std::min(a.y+a.h, b.y+b.h);
  float iw = std::max(0.f, x2-x1), ih = std::max(0.f, y2-y1);
  float inter = iw*ih, uni = a.w*a.h + b.w*b.h - inter;
  return uni>0 ? inter/uni : 0.f;
}

class GlobalIdFuserImpl : public GlobalIdFuser {
  FuserParams p_;
  std::atomic<GlobalId> next_{1};
  struct G { GlobalObject o; };
  std::vector<G> active_; // маленький пул — простая линейная проверка
public:
  explicit GlobalIdFuserImpl(const FuserParams& p):p_(p){}

  void Ingest(const CamId& cam, std::vector<Track>&& tracks, usec_t ts) override {
    // TTL cleanup
    const usec_t ttl_us = (usec_t)p_.ttl_ms*1000;
    active_.erase(std::remove_if(active_.begin(), active_.end(),
      [&](const G& g){ return ts > g.o.last_seen_us + ttl_us; }), active_.end());

    for (auto& t : tracks){
      // поиск совпадения
      int best=-1; float biou=0.f;
      for (size_t i=0;i<active_.size();++i){
        const auto dt = (int64_t)ts - (int64_t)active_[i].o.last_seen_us;
        if (std::abs(dt) > p_.time_gate_ms*1000) continue;
        float iou = IoU(active_[i].o.last_track.box, t.box);
        if (iou>biou && iou>=p_.iou_gate){ biou=iou; best=(int)i; }
      }
      if (best!=-1){
        // обновляем существующий глобальный объект
        active_[best].o.cam_id = cam;
        active_[best].o.last_track = t;
        active_[best].o.last_seen_us = ts;
      } else {
        // создаём новый
        G g{};
        g.o.id = next_++;
        g.o.cam_id = cam;
        g.o.last_track = t;
        g.o.last_seen_us = ts;
        active_.push_back(std::move(g));
      }
    }
  }

  std::vector<GlobalObject> Snapshot() const override {
    std::vector<GlobalObject> v; v.reserve(active_.size());
    for (auto& g: active_) v.push_back(g.o);
    return v;
  }

  void Reset() override { active_.clear(); }
};

std::unique_ptr<GlobalIdFuser> CreateGlobalIdFuser(const FuserParams& p) {
  return std::make_unique<GlobalIdFuserImpl>(p);
}
} // namespace mc
