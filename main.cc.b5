// ========================== main.cc (полная версия) ==========================

#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "web_server.h"   // из include/web_server.h (namespace mc)

// ----------------------------------------------------------------------------
// Глобальный флаг завершения
// ----------------------------------------------------------------------------
static std::atomic<bool> g_run{true};

// ----------------------------------------------------------------------------
// Обработчик сигналов
// ----------------------------------------------------------------------------
static void on_signal(int) {
  g_run.store(false);
}

// ============================================================================
// parse_cli function start
// ============================================================================
struct CliOptions {
  std::string config_path;   // то, что пришло в --config
  int http_fps_limit = 20;   // по умолчанию 20 (можно переопределить)
};

static CliOptions parse_cli(int argc, char** argv) {
  CliOptions opt;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--config" && i + 1 < argc) {
      opt.config_path = argv[++i];
    } else if (a == "--http-fps-limit" && i + 1 < argc) {
      try {
        opt.http_fps_limit = std::stoi(argv[++i]);
      } catch (...) {
        std::cerr << "Bad value for --http-fps-limit, using default 20\n";
        opt.http_fps_limit = 20;
      }
    }
  }
  return opt;
}
// ============================================================================
// parse_cli function end
// ============================================================================


// ============================================================================
// load_config_minimal function start
// ----------------------------------------------------------------------------
// Наша минималка: проверяем, что конфиг существует. Если путь не задан,
// пробуем несколько запасных вариантов. Порт берём 8080, а активную камеру
// по умолчанию "cam1" (как раньше). Полный парсинг JSON прикрутим позже.
// ============================================================================
struct AppConfigMinimal {
  int http_port = 8080;
  int http_fps_limit = 20;
  std::string display_camera = "cam1";
  std::string used_config_path; // куда в итоге ткнулись
};

static bool file_exists(const std::string& p) {
  std::ifstream f(p);
  return f.good();
}

static bool load_config_minimal(const std::string& cli_path,
                                AppConfigMinimal& out,
                                std::string& err) {
  std::vector<std::string> candidates;
  if (!cli_path.empty()) candidates.push_back(cli_path);
  // fallback-ы рядом с бинарём/примером
  candidates.push_back("config/config.json");
  candidates.push_back("../config/config.json");

  for (const std::string& p : candidates) {
    if (file_exists(p)) {
      out.used_config_path = p;
      return true;
    }
  }
  err = "Config load failed: --config";
  return false;
}
// ============================================================================
// load_config_minimal function end
// ============================================================================


// ============================================================================
// AppRuntime class start
// ----------------------------------------------------------------------------
// Управляет жизненным циклом HTTP-сервера. Камеры/детектор подключим на
// следующем шаге, сейчас только HTTP + коллбек переключения камер.
// ============================================================================
class AppRuntime {
public:
  AppRuntime() = default;
  ~AppRuntime() { stop(); }

  bool start(const AppConfigMinimal& cfg) {
    using namespace mc;

    // Подготовим и запустим HTTP
    mc::WebServerConfig http_cfg;
    http_cfg.port = cfg.http_port;
    http_cfg.fps_limit = cfg.http_fps_limit;

    http_ = mc::CreateWebServer();
    if (!http_) {
      std::cerr << "[bootstrap] HTTP: CreateWebServer() failed\n";
      return false;
    }

    // Коллбек из UI: переключение активной камеры
    http_->SetOnSwitchCamera([this](const mc::CamId& cam) {
      std::cout << "[http] switch camera -> " << cam << std::endl;
      active_camera_ = cam;  // на будущее, когда подключим CameraManager
    });

    if (!http_->Start(http_cfg)) {
      std::cerr << "[bootstrap] HTTP start FAILED (port="
                << http_cfg.port << ")\n";
      return false;
    }

    std::cout << "[bootstrap] HTTP: port=" << http_cfg.port
              << " fps_limit=" << http_cfg.fps_limit
              << " display='" << active_camera_ << "'\n";
    return true;
  }

  void stop() {
    if (http_) {
      http_->Stop();
      http_.reset();
    }
  }

  const std::string& active_camera() const { return active_camera_; }

private:
  std::unique_ptr<mc::WebServer> http_;
  std::string active_camera_ = "cam1";
};
// ============================================================================
// AppRuntime class end
// ============================================================================


// ============================================================================
// main function start
// ============================================================================
int main(int argc, char** argv) {
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  // CLI
  CliOptions cli = parse_cli(argc, argv);

  // Загрузка/проверка мини-конфига
  AppConfigMinimal cfg;
  cfg.http_fps_limit = cli.http_fps_limit;  // переопределяем с CLI
  std::string err;
  if (!load_config_minimal(cli.config_path, cfg, err)) {
    std::cerr << err << std::endl;
    return 1;
  }

  std::cout << "Loading config: " << cfg.used_config_path << std::endl;
  std::cout << "Config OK (port=" << cfg.http_port
            << ", http_fps_limit=" << cfg.http_fps_limit
            << ", display_camera=\"" << cfg.display_camera << "\")\n";

  // Старт приложения
  std::cout << "[bootstrap] App started.\n";
  AppRuntime app;
  // стартуем с display_camera из конфига (для логов и будущей связки с камерами)
  // сейчас — только логика в AppRuntime
  if (!cfg.display_camera.empty()) {
    // просто инициализируем поле через временный старт/стоп? не нужно,
    // сразу стартуем: AppRuntime сам выведет display
  }

  if (!app.start(cfg)) {
    return 2;
  }

  // Главный цикл
  while (g_run.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Остановка
  app.stop();
  return 0;
}
// ============================================================================
// main function end
// ============================================================================

