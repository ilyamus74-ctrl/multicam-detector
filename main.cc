// ========== includes start ==========
#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

// если будешь сразу подключать свои модули — раскомментируй нужные include'ы
//#include "config_loader.h"
#include "camera_manager.h"
#include "npu_worker_pool.h"
#include "global_id_fuser.h"
#include "tracker_sort.h"
#include "web_server.h"
// ========== includes end ==========



// ========== globals start ==========
static std::atomic<bool> g_run{true};

static void signal_handler(int) {
  g_run.store(false);
}
// ========== globals end ==========



// ========== small utils start ==========
static bool file_exists(const std::string& p) {
  std::error_code ec;
  return std::filesystem::exists(p, ec);
}

struct CliOptions {
  std::string config_path = "config/config.json"; // дефолт
  int http_fps_limit_cli = -1;                    // -1 = не переопределять
  bool show_help = false;
};

// Универсальный парсер: понимает "--opt=val" и "--opt val" + позиционный путь
static CliOptions parse_cli(int argc, char** argv) {
  CliOptions opt;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];

    auto read_value = [&](std::string& out)->bool {
      auto eq = a.find('=');
      if (eq != std::string::npos) { out = a.substr(eq + 1); return true; }
      if (i + 1 < argc) { out = argv[++i]; return true; }
      return false;
    };

    if (a == "-h" || a == "--help") {
      opt.show_help = true;
      break;
    } else if (a.rfind("--config", 0) == 0) {
      std::string v;
      if (!read_value(v)) {
        std::cerr << "Missing value for --config\n";
        opt.show_help = true;
        break;
      }
      opt.config_path = v;
    } else if (a.rfind("--http-fps-limit", 0) == 0) {
      std::string v;
      if (!read_value(v)) {
        std::cerr << "Missing value for --http-fps-limit\n";
        opt.show_help = true;
        break;
      }
      try {
        opt.http_fps_limit_cli = std::stoi(v);
      } catch (...) {
        std::cerr << "Bad value for --http-fps-limit: " << v << "\n";
        opt.show_help = true;
        break;
      }
    } else if (!a.empty() && a[0] != '-') {
      // позиционный аргумент считаем путём к конфигу
      opt.config_path = a;
    } else {
      std::cerr << "Unknown option: " << a << "\n";
      opt.show_help = true;
      break;
    }
  }

  // Фоллбэки на типичные пути, если указанный не найден
  if (!opt.show_help && !file_exists(opt.config_path)) {
    for (const char* fb : {"config/config.json", "../config/config.json"}) {
      if (file_exists(fb)) { opt.config_path = fb; break; }
    }
  }
  return opt;
}

static void print_usage(const char* argv0) {
  std::cout <<
    "Usage:\n"
    "  " << argv0 << " [--config=PATH | PATH] [--http-fps-limit=N]\n\n"
    "Options:\n"
    "  --config PATH          Путь к config.json (можно позиционно)\n"
    "  --http-fps-limit N     Ограничение FPS для HTTP-выдачи (CLI override)\n"
    "  -h, --help             Показать эту справку\n";
}
// ========== small utils end ==========



// ========== config-load helper start ==========
// Минимальная конфигурация, которой хватает main'у
struct AppConfigMinimal {
  int http_port = 8080;
  int http_fps_limit = 20;
  std::string display_camera = "cam1";
};

static bool load_config_minimal(const std::string& path,
                                AppConfigMinimal& /*out_cfg*/,
                                std::string& err) {
  // Заглушка: просто проверим наличие файла, дальше пусть работают дефолты
  if (!file_exists(path)) { err = "file not found: " + path; return false; }
  return true;
}
// ========== config-load helper end ==========



// ========== bootstrap start ==========
// Здесь запускаем реальные компоненты приложения
struct AppRuntime {
  WebServer http;   // <-- твой веб-сервер
  bool started = false;

  bool start(const AppConfigMinimal& cfg) {
    // Вариант 1 (частый): start(port, fps_limit, display_camera)
    if (!http.start(cfg.http_port, cfg.http_fps_limit, cfg.display_camera)) {
      std::cerr << "[bootstrap] WebServer start() failed on port "
                << cfg.http_port << "\n";
      return false;
    }

    started = true;
    std::cout << "[bootstrap] WebServer listening on port "
              << cfg.http_port << "\n";
    return true;
  }

  void stop() {
    if (!started) return;
    http.stop();
    started = false;
    std::cout << "[bootstrap] App stopped.\n";
  }
};
// ПРИМЕЧАНИЕ: если у тебя сигнатура другая (например,
//   bool start(int port, int fps_limit);
//   или   bool start(int port, int fps_limit, const std::string&, CameraManager*)
// — просто поправь строку вызова выше под твою `web_server.h`.
// ========== bootstrap end ==========



// ========== main function start ==========
int main(int argc, char** argv) {
  // 0) Сигналы на аккуратное завершение
  std::signal(SIGINT,  signal_handler);
  std::signal(SIGTERM, signal_handler);

  // 1) Разбор аргументов
  CliOptions cli = parse_cli(argc, argv);
  if (cli.show_help) {
    print_usage(argv[0]);
    return cli.http_fps_limit_cli == -1 ? 0 : 1; // если help из-за ошибки — вернём 1
  }

  std::cout << "Loading config: " << cli.config_path << "\n";
  if (!file_exists(cli.config_path)) {
    std::cerr << "Config load failed: " << cli.config_path << "\n";
    return 1;
  }

  // 2) Загрузка конфига
  AppConfigMinimal cfg{};
  std::string err;
  if (!load_config_minimal(cli.config_path, cfg, err)) {
    std::cerr << "Config load failed: " << err << "\n";
    return 1;
  }

  // Переопределение http_fps_limit из CLI, если задано
  if (cli.http_fps_limit_cli >= 0) {
    cfg.http_fps_limit = cli.http_fps_limit_cli;
  }

  std::cout << "Config OK (port=" << cfg.http_port
            << ", http_fps_limit=" << cfg.http_fps_limit
            << ", display_camera=\"" << cfg.display_camera << "\")\n";

  // 3) Bootstrap: запуск компонентов
  AppRuntime app;
  if (!app.start(cfg)) {
    std::cerr << "Failed to start application components.\n";
    return 1;
  }

  // 4) Основной цикл
  while (g_run.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 5) Остановка
  app.stop();
  return 0;
}
// ========== main function end ==========
