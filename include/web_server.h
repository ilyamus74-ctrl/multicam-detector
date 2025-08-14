#pragma once
#include <string>
#include <vector>
#include <functional>
#include "common_types.h"

namespace mc {

struct WebServerConfig {
  int port = 8080;
  int fps_limit = 20;
};

class WebServer {
public:
  using OnSwitchCamera = std::function<void(const CamId&)>;

  virtual ~WebServer() = default;

  virtual bool Start(const WebServerConfig& cfg) = 0;
  virtual void Stop() = 0;

  // Картинка активной камеры (сервер сам тянет последний кадр/рендер)
  virtual void PushRenderedFrame(const CamId& cam, const void* jpg_data, size_t jpg_size, usec_t ts_us) = 0;

  // Агрегированные метаданные (по всем камерам, global_id)
  virtual void PushMeta(const std::vector<GlobalObject>& objs, usec_t ts_us) = 0;

  // Кнопки в UI → переключение активной камеры
  virtual void SetOnSwitchCamera(OnSwitchCamera cb) = 0;
};

std::unique_ptr<WebServer> CreateWebServer();

} // namespace mc
