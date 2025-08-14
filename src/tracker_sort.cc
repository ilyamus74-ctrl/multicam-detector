#include "tracker_sort.h"
#include <unordered_map>
#include <algorithm>
#include <atomic>

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

class TrackerSORTImpl : public TrackerSORT {
  SortParams p_;
  struct T { Track t; int age=0; int hits=0; };
  std::vector<T> tr_;
  std::atomic<int> next_id_{1};
public:
  explicit TrackerSORTImpl(const SortParams& p):p_(p){}

  std::vector<Track> Update(const std::vector<Detection>& dets, usec_t ts) override {
    // match
    std::vector<int> det_assigned(dets.size(), -1);
    for (size_t i=0;i<tr_.size();++i){
      int best=-1; float biou=0;
      for (size_t j=0;j<dets.size();++j){
        if (det_assigned[j]!=-1) continue;
        float iou=IoU(tr_[i].t.box, dets[j].box);
        if (iou>biou && iou>=p_.iou_thresh){ biou=iou; best=(int)j; }
      }
      if (best!=-1){
        tr_[i].t.box = dets[best].box;
        tr_[i].t.ts_us = ts;
        tr_[i].age = 0;
        tr_[i].hits++;
        det_assigned[best]=(int)i;
      } else {
        tr_[i].age++;
      }
    }
    // new tracks
    for (size_t j=0;j<dets.size();++j){
      if (det_assigned[j]==-1){
        T nt{};
        nt.t.track_id = next_id_++;
        nt.t.box = dets[j].box;
        nt.t.ts_us = ts;
        nt.hits=1;
        tr_.push_back(std::move(nt));
      }
    }
    // prune
    tr_.erase(std::remove_if(tr_.begin(), tr_.end(),
      [&](const T& x){ return x.age>p_.max_age; }), tr_.end());

    // output
    std::vector<Track> out;
    out.reserve(tr_.size());
    for (auto& x: tr_){
      if (x.hits>=p_.min_hits) out.push_back(x.t);
    }
    return out;
  }

  std::vector<Track> LiveTracks() const override {
    std::vector<Track> out; out.reserve(tr_.size());
    for (auto& x: tr_) out.push_back(x.t);
    return out;
  }
};

std::unique_ptr<TrackerSORT> CreateTrackerSORT(const SortParams& p){
  return std::make_unique<TrackerSORTImpl>(p);
}
} // namespace mc
