// main.cc
#include <atomic>
#include <csignal>
#include <cctype>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "include/web_server.h"
#include "include/camera_manager.h"
#include "include/common_types.h"

using namespace std::chrono_literals;

struct CliOptions {
  std::string config_path;
  int http_fps_limit_cli = -1;   // <0 — не задано, используем из файла
};

struct HttpMinimal {
  int         port = 8080;
  int         fps_limit = 20;
  std::string display_camera;
};

// -------------------- small helpers start --------------------
static bool file_exists(const std::string& p) {
  std::ifstream f(p);
  return f.good();
}

static std::string read_all(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  if (!f) return {};
  std::string s;
  f.seekg(0, std::ios::end);
  s.resize(static_cast<size_t>(f.tellg()));
  f.seekg(0, std::ios::beg);
  f.read(s.data(), static_cast<std::streamsize>(s.size()));
  return s;
}

static void trim_spaces(std::string& s) {
  size_t a = 0, b = s.size();
  while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
  while (b > a && std::isspace(static_cast<unsigned char>(s[b-1]))) --b;
  s.assign(s.data()+a, b-a);
}
// -------------------- small helpers end ----------------------

// -------------------- load_http_minimal start ----------------
// Минимальный «парсер» только блока "http" из JSON.
// Без сторонних библиотек: вырезаем подстроку { ... } после ключа "http"
// и внутри грубо находим значения 3-х полей.
static bool load_http_minimal(const std::string& path, HttpMinimal& out, std::string& err) {
  if (path.empty() || !file_exists(path)) {
    err = "config not found: " + path;
    return false;
  }
  const std::string content = read_all(path);
  if (content.empty()) {
    err = "config is empty or not readable: " + path;
    return false;
  }

  auto npos = std::string::npos;
  auto http_key = content.find("\"http\"");
  if (http_key == npos) {
    // Нет блока "http" — не ошибка, просто используем значения по умолчанию
    return true;
  }
  auto brace_open = content.find('{', http_key);
  if (brace_open == npos) return true;

  size_t i = brace_open + 1;
  int balance = 1;
  for (; i < content.size() && balance > 0; ++i) {
    if (content[i] == '{') ++balance;
    else if (content[i] == '}') --balance;
  }
  if (balance != 0) {
    err = "malformed http object (braces mismatch)";
    return false;
  }
  const size_t brace_close = i - 1;
  std::string http_obj = content.substr(brace_open + 1, brace_close - brace_open - 1);

  auto find_int = [&](const char* key, int& dst) {
    std::string kq = std::string("\"") + key + "\"";
    auto kpos = http_obj.find(kq);
    if (kpos == npos) return;
    auto colon = http_obj.find(':', kpos);
    if (colon == npos) return;
    size_t p = colon + 1;
    while (p < http_obj.size() && std::isspace(static_cast<unsigned char>(http_obj[p]))) ++p;
    char* endp = nullptr;
    long v = std::strtol(http_obj.c_str() + p, &endp, 10);
    if (endp && endp != http_obj.c_str() + p) dst = static_cast<int>(v);
  };

  auto find_string = [&](const char* key, std::string& dst) {
    std::string kq = std::string("\"") + key + "\"";
    auto kpos = http_obj.find(kq);
    if (kpos == npos) return;
    auto colon = http_obj.find(':', kpos);
    if (colon == npos) return;
    size_t q1 = http_obj.find('"', colon + 1);
    if (q1 == npos) return;
    size_t q2 = http_obj.find('"', q1 + 1);
    if (q2 == npos) return;
    dst = http_obj.substr(q1 + 1, q2 - q1 - 1);
    trim_spaces(dst);
  };

  find_int("port", out.port);
  find_int("http_fps_limit", out.fps_limit);
  find_string("display_camera", out.display_camera);

  return true;
}
// -------------------- load_http_minimal end ------------------

// -------------------- parse_cli start ------------------------
static CliOptions parse_cli(int argc, char** argv) {
  CliOptions opt;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if ((a == "--config" || a == "-c") && i + 1 < argc) {
      opt.config_path = argv[++i];
    } else if (a == "--http-fps-limit" && i + 1 < argc) {
      opt.http_fps_limit_cli = std::atoi(argv[++i]);
    }
  }
  if (opt.config_path.empty()) {
    // Фолбэки (как раньше)
    for (const std::string fb : {
            "config/config.json",
            "../config/config.json",
            "../../config/config.json" }) {
      if (file_exists(fb)) { opt.config_path = fb; break; }
    }
  }
  return opt;
}
// -------------------- parse_cli end --------------------------

// -------------------- AppRuntime start -----------------------
class AppRuntime {
public:
  bool Start(const std::string& config_path, const HttpMinimal& http_cfg) {
    // HTTP сервер
    mc::WebServerConfig wcfg;
    wcfg.port = http_cfg.port;
    wcfg.fps_limit = http_cfg.fps_limit;

    if (!http_->Start(wcfg)) {
      std::cerr << "[bootstrap] HTTP start failed on port " << wcfg.port << "\n";
      return false;
    }

    // Коллбек из UI: переключение камеры
    http_->SetOnSwitchCamera([this](const mc::CamId& cam){
      std::cout << "[http] switch camera -> " << cam << "\n";
      cams_->SetDisplayCamera(cam);
    });

    // Если в конфиге задана стартовая камера — установим
    if (!http_cfg.display_camera.empty())
      cams_->SetDisplayCamera(http_cfg.display_camera);

    std::cout << "[bootstrap] HTTP: port=" << wcfg.port
              << " fps_limit=" << wcfg.fps_limit
              << " display='" << http_cfg.display_camera << "'\n";

    // Здесь можно добавить запуск менеджера камер, когда будем готовы
    // (Start(AppConfig) и т.п.)
    return true;
  }

  void Stop() {
    if (http_) http_->Stop();
    // останавливать камеры, если запущены
  }

  AppRuntime()
    : http_(mc::CreateWebServer())
    , cams_(mc::CreateCameraManager())
  {}

private:
  std::unique_ptr<mc::WebServer>     http_;
  std::unique_ptr<mc::CameraManager> cams_;
};
// -------------------- AppRuntime end -------------------------

static std::atomic<bool> g_run{true};
static void on_sigint(int){ g_run = false; }

// -------------------- main function start --------------------
int main(int argc, char** argv) {
  std::signal(SIGINT, on_sigint);

  CliOptions cli = parse_cli(argc, argv);
  if (cli.config_path.empty()) {
    std::cerr << "Config load failed: --config\n";
    return 2;
  }

  std::cout << "Loading config: " << cli.config_path << "\n";

  HttpMinimal http_cfg;
  std::string err;
  if (!load_http_minimal(cli.config_path, http_cfg, err)) {
    std::cerr << "Config load failed: " << err << "\n";
    return 2;
  }

  if (cli.http_fps_limit_cli >= 0)
    http_cfg.fps_limit = cli.http_fps_limit_cli;

  std::cout << "Config OK (port=" << http_cfg.port
            << ", http_fps_limit=" << http_cfg.fps_limit
            << ", display_camera=\"" << (http_cfg.display_camera.empty() ? ": " : http_cfg.display_camera) << "\")\n";

  AppRuntime app;
  if (!app.Start(cli.config_path, http_cfg)) return 3;

  std::cout << "[bootstrap] App started.\n";
  while (g_run) std::this_thread::sleep_for(100ms);

  app.Stop();
  return 0;
}
// -------------------- main function end ----------------------



