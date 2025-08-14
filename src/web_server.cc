#include "web_server.h"
#include "httplib.h"
#include "nlohmann/json.hpp"
#include <mutex>
#include <atomic>

using nlohmann::json;
namespace mc {
class WebServerImpl : public WebServer {
  httplib::Server srv_;
  std::thread th_;
  std::atomic<bool> run_{false};
  WebServerConfig cfg_;
  OnSwitchCamera on_switch_;
  std::mutex m_;
  // последние метаданные
  json last_meta_ = json::array();
  CamId last_cam_;
public:
  bool Start(const WebServerConfig& cfg) override {
    cfg_ = cfg;
    // GET /meta
    srv_.Get("/meta", [this](const httplib::Request&, httplib::Response& res){
      std::scoped_lock lk(m_);
      res.set_content(last_meta_.dump(), "application/json");
    });
    // POST /switch?cam=ID
    srv_.Post("/switch", [this](const httplib::Request& req, httplib::Response& res){
      auto id = req.get_param_value("cam");
      if (on_switch_) on_switch_(id);
      res.set_content("{\"ok\":true}", "application/json");
    });
    // Простой индекс (кнопки)
    srv_.Get("/", [](const httplib::Request&, httplib::Response& res){
      static const char* html =
        "<html><body>"
        "<h3>multicam</h3>"
        "<button onclick=\"fetch('/switch?cam=cam0',{method:'POST'})\">Вкл cam0</button>"
        "<button onclick=\"fetch('/switch?cam=cam1',{method:'POST'})\">Вкл cam1</button>"
        "<pre id='m'></pre>"
        "<script>setInterval(async()=>{let r=await fetch('/meta'); m.textContent=await r.text();},500);</script>"
        "</body></html>";
      res.set_content(html, "text/html");
    });

    run_ = true;
    th_ = std::thread([this]{
      srv_.listen("0.0.0.0", cfg_.port);
    });
    return true;
  }
  void Stop() override {
    if (!run_) return;
    run_=false; srv_.stop();
    if (th_.joinable()) th_.join();
  }
  void PushRenderedFrame(const CamId& cam, const void* /*jpg*/, size_t /*sz*/, usec_t /*ts*/) override {
    std::scoped_lock lk(m_); last_cam_ = cam;
  }
  void PushMeta(const std::vector<GlobalObject>& objs, usec_t /*ts*/) override {
    json a = json::array();
    for (auto& o: objs){
      a.push_back({
        {"id", o.id},
        {"cam", o.cam_id},
        {"x", o.last_track.box.x},
        {"y", o.last_track.box.y},
        {"w", o.last_track.box.w},
        {"h", o.last_track.box.h}
      });
    }
    std::scoped_lock lk(m_);
    last_meta_ = std::move(a);
  }
  void SetOnSwitchCamera(OnSwitchCamera cb) override { on_switch_ = std::move(cb); }
};

std::unique_ptr<WebServer> CreateWebServer(){
  return std::make_unique<WebServerImpl>();
}
} // namespace mc
