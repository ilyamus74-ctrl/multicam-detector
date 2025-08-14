#include "camera_manager.h"
#include "v4l2_profiles.h"
#include "nlohmann/json.hpp"
#include <fstream>

using nlohmann::json;
namespace mc {

static V4L2Profile readProfile(const json& j) {
  V4L2Profile p{};
  p.name = j.value("name","profile");
  p.exposure_abs = j.value("exposure_abs", 160);
  p.gain = j.value("gain", 8);
  p.gamma = j.value("gamma", 120);
  p.saturation = j.value("saturation", 128);
  p.force_grey = j.value("force_grey", false);
  return p;
}

bool LoadConfigFromFile(const std::string& path, AppConfig* out) {
  std::ifstream f(path);
  if (!f) return false;
  json j; f >> j;

  out->http.port = j["http"].value("port", 8080);
  out->http.http_fps_limit = j["http"].value("fps_limit", 20);
  out->http.display_camera = j["http"].value("display_camera", "");

  // profiles
  auto pj = j["profiles"];
  out->profiles.bright = readProfile(pj["bright"]);
  out->profiles.indoor = readProfile(pj["indoor"]);
  out->profiles.dark   = readProfile(pj["dark"]);
  out->profiles.sw.metric = pj["switch"].value("metric","mean_luma");
  out->profiles.sw.bright_if = pj["switch"].value("bright_if", 180);
  out->profiles.sw.dark_if   = pj["switch"].value("dark_if", 60);
  out->profiles.sw.hysteresis= pj["switch"].value("hysteresis", 10);

  // cameras
  out->cameras.clear();
  for (auto& cj : j["cameras"]) {
    CameraConfig c{};
    c.id   = cj.value("id","");
    c.match.by_id_contains = cj["match"].value("by_id_contains", "");
    c.w    = cj.value("width", 1280);
    c.h    = cj.value("height",720);
    c.fps  = cj.value("fps", 30);
    c.pixfmt = cj.value("pixfmt","MJPG");
    c.npu_worker = cj.value("npu_worker", 0);
    c.auto_profiles = cj.value("auto_profiles", true);
    out->cameras.push_back(std::move(c));
  }
  return true;
}

} // namespace mc
