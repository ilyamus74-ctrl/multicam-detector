// YOLOv8 HTTP web server (V4L2 MJPEG -> TurboJPEG -> RKNN)
// Flags:
//   --dev /dev/videoX
//   --port N
//   --size WxH
//   --cap-fps N
//   --buffers N
//   --jpeg-quality 30..95
//   --http-fps-limit N
//   --fps
//   --npu-core auto|0|1|2|01|012

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <linux/videodev2.h>

#include "turbojpeg.h"
#include "image_drawing.h"
#include "yolov8.h"
#include "common.h"
#include "image_utils.h"
#include "file_utils.h"
#include "httplib.h"
#include "nlohmann/json.hpp"

//debug start
#include <chrono>

struct StageAcc {
  double cap=0, prep=0, infer=0, draw=0, enc=0, loop=0;
  int n=0;
  void add_print_and_reset(int every=30) {
    n++;
    if (n % every == 0) {
      printf("TIMES(avg %d): cap=%.1fms prep=%.1fms infer=%.1fms draw=%.1fms enc=%.1fms | loop=%.1fms\n",
             every, cap/every, prep/every, infer/every, draw/every, enc/every, loop/every);
      cap=prep=infer=draw=enc=loop=0; n=0;
    }
  }
};

#define TICK(x) auto x##_t0 = std::chrono::steady_clock::now()
#define TOCK(x,acc_field) acc_field += std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now() - x##_t0).count()
//debug end


using json = nlohmann::json;
using namespace httplib;
using Clock = std::chrono::high_resolution_clock;

// ---------- CLI ----------
struct Args {
    std::string model;
    std::string dev = "/dev/video0";
    int port = 8080;
    int cap_w = 640, cap_h = 480;
    int cap_fps = 30;
    int buffers = 3;
    int jpeg_q = 70;
    int http_fps_limit = 0;
    bool show_fps = false;
    std::string npu_core = "auto"; // auto|0|1|2|01|012
};

static bool parseSize(const std::string& s, int& w, int& h) {
    auto x = s.find('x');
    if (x == std::string::npos) return false;
    try { w = std::stoi(s.substr(0, x)); h = std::stoi(s.substr(x+1)); return w>0 && h>0; }
    catch (...) { return false; }
}

static bool isNumber(const char* s) {
    if (!s || !*s) return false;
    for (const char* p = s; *p; ++p) if (*p < '0' || *p > '9') return false;
    return true;
}

static Args parseArgs(int argc, char** argv) {
    Args a;
    if (argc >= 2) a.model = argv[1];

    // совместимость: если 2-й позиционный — число, это порт; если /dev/video*, это dev
    if (argc >= 3 && argv[2][0] != '-') {
        std::string s2 = argv[2];
        if (s2.rfind("/dev/video", 0) == 0) a.dev = s2;
        else if (isNumber(argv[2])) a.port = atoi(argv[2]);
    }
    // остальные — только именованные
    for (int i = 2; i < argc; ++i) {
        std::string k = argv[i];
        auto need = [&](int more){ return i+more < argc; };
        if (k == "--dev" && need(1)) a.dev = argv[++i];
        else if (k == "--port" && need(1) && isNumber(argv[i+1])) a.port = atoi(argv[++i]);
        else if (k == "--size" && need(1)) { int w=0,h=0; if (parseSize(argv[++i], w, h)){ a.cap_w=w; a.cap_h=h; } }
        else if (k == "--cap-fps" && need(1)) a.cap_fps = atoi(argv[++i]);
        else if (k == "--buffers" && need(1)) a.buffers = std::max(1, atoi(argv[++i]));
        else if (k == "--jpeg-quality" && need(1)) a.jpeg_q = std::max(30, std::min(95, atoi(argv[++i])));
        else if (k == "--http-fps-limit" && need(1)) a.http_fps_limit = std::max(0, atoi(argv[++i]));
        else if (k == "--fps") a.show_fps = true;
        else if (k == "--npu-core" && need(1)) a.npu_core = argv[++i]; // auto|0|1|2|01|012
    }
    return a;
}

// ---------- utils ----------
static inline const char* fourcc_to_str(__u32 f, char s[5]) {
    s[0] = (char)(f & 0xFF);
    s[1] = (char)((f >> 8) & 0xFF);
    s[2] = (char)((f >> 16) & 0xFF);
    s[3] = (char)((f >> 24) & 0xFF);
    s[4] = 0;
    return s;
}

static rknn_core_mask npu_mask_from_string(const std::string& s) {
    if (s == "auto") return RKNN_NPU_CORE_AUTO;
    if (s == "0")    return RKNN_NPU_CORE_0;
    if (s == "1")    return RKNN_NPU_CORE_1;
    if (s == "2")    return RKNN_NPU_CORE_2;
    if (s == "01" || s == "0_1")                 return RKNN_NPU_CORE_0_1;
    if (s == "012" || s == "0_1_2" || s == "all") return RKNN_NPU_CORE_0_1_2;
    return RKNN_NPU_CORE_AUTO;
}

// ---------- server ----------
class YOLOWebServer {
private:
    rknn_app_context_t rknn_app_ctx{};
    Server server;
    bool model_initialized = false;

    // cfg
    std::string model_path;
    int server_port = 8080;
    std::string cam_dev = "/dev/video0";
    int cam_w_req=640, cam_h_req=480, cam_fps_req=30, cam_buffers=3;
    int jpeg_q = 70;
    int http_fps_limit = 0;
    bool show_fps = false;
    rknn_core_mask npu_mask = RKNN_NPU_CORE_AUTO;

    // camera
    struct CamBuffer { void* start=nullptr; size_t length=0; };
    int cam_fd = -1;
    std::vector<CamBuffer> cam_bufs;
    std::thread cam_thread{};
    std::atomic<bool> cam_running{false};
    int cam_w=0, cam_h=0, cam_fps=0;

    // shared
    std::mutex infer_mtx;
    std::mutex frame_mtx;
    std::condition_variable frame_cv;
    std::vector<uint8_t> last_jpeg;
    json last_meta;

public:
    YOLOWebServer(const Args& a)
        : model_path(a.model), server_port(a.port),
          cam_dev(a.dev), cam_w_req(a.cap_w), cam_h_req(a.cap_h),
          cam_fps_req(a.cap_fps), cam_buffers(a.buffers),
          jpeg_q(a.jpeg_q), http_fps_limit(a.http_fps_limit),
          show_fps(a.show_fps), npu_mask(npu_mask_from_string(a.npu_core)) {}

    ~YOLOWebServer() { cleanup(); }

    bool initialize() {
        if (init_post_process() != 0) {
            fprintf(stderr, "init_post_process failed\n");
            return false;
        }
        if (init_yolov8_model(model_path.c_str(), &rknn_app_ctx) != 0) {
            fprintf(stderr, "init_yolov8_model failed: %s\n", model_path.c_str());
            deinit_post_process();
            return false;
        }
        // Устанавливаем маску NPU по флагу
        {
            int ret_mask = rknn_set_core_mask(rknn_app_ctx.rknn_ctx, npu_mask);
            if (ret_mask != RKNN_SUCC) fprintf(stderr, "warn: rknn_set_core_mask(%d) failed: %d\n", npu_mask, ret_mask);
            else                        fprintf(stderr, "rknn core mask set: %d\n", npu_mask);
        }
        model_initialized = true;

        server.set_keep_alive_max_count(100);
        server.set_read_timeout(5, 0);
        server.set_write_timeout(5, 0);
        server.set_payload_max_length(1 * 1024 * 1024);

        setupRoutes();

        if (initCamera()) {
            cam_thread = std::thread(&YOLOWebServer::cameraLoop, this);
        } else {
            fprintf(stderr, "WARN: camera init failed (%s). Stream endpoints -> 503\n", cam_dev.c_str());
        }
        return true;
    }

    void run() {
        printf("Starting YOLOv8 Web Server on port %d\n", server_port);
        printf("UI: http://localhost:%d\n", server_port);
        server.listen("0.0.0.0", server_port);
    }

    void stop() { server.stop(); }

private:
    void cleanup() {
        server.stop();
        cam_running = false;
        frame_cv.notify_all();
        if (cam_thread.joinable()) cam_thread.join();
        deinitCamera();
        if (model_initialized) {
            deinit_post_process();
            release_yolov8_model(&rknn_app_ctx);
            model_initialized = false;
        }
    }

    // ---------- HTTP ----------
    void setupRoutes() {
        server.set_pre_routing_handler([](const Request&, Response& res) {
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
            res.set_header("Access-Control-Allow-Headers", "Content-Type, Authorization");
            return Server::HandlerResponse::Unhandled;
        });
        server.Options(".*", [](const Request&, Response&){});
        server.set_mount_point("/", "./web");

        server.Get("/api/health", [this](const Request&, Response& res) {
            json j;
            j["status"] = model_initialized ? "ready" : "not_ready";
            j["model_path"] = model_path;
            j["streaming"] = cam_running.load();
            j["cap_req_w"] = cam_w_req; j["cap_req_h"] = cam_h_req; j["cap_req_fps"] = cam_fps_req;
            j["cap_real_w"] = cam_w; j["cap_real_h"] = cam_h; j["cap_real_fps"] = cam_fps;
            j["jpeg_q"] = jpeg_q;
            j["npu_mask"] = npu_mask;
            res.set_content(j.dump(), "application/json");
        });

        server.Get("/api/model-info", [this](const Request&, Response& res) {
            if (!model_initialized) { res.status = 503; res.set_content("{\"error\":\"Model not initialized\"}", "application/json"); return; }
            json j;
            j["model_width"] = rknn_app_ctx.model_width;
            j["model_height"] = rknn_app_ctx.model_height;
            j["model_channel"] = rknn_app_ctx.model_channel;
            j["is_quantized"] = rknn_app_ctx.is_quant;
            j["input_count"] = rknn_app_ctx.io_num.n_input;
            j["output_count"] = rknn_app_ctx.io_num.n_output;
            res.set_content(j.dump(), "application/json");
        });

        server.Post("/api/detect", [](const Request&, Response& res) {
            res.status = 501;
            res.set_content("{\"error\":\"/api/detect disabled; use /api/stream.mjpg\"}", "application/json");
        });

        server.Get("/api/stream.mjpg", [this](const Request&, Response& res) {
            if (!cam_running.load()) { res.status = 503; res.set_content("camera not running", "text/plain"); return; }
            res.set_header("Cache-Control", "no-store, no-cache, must-revalidate");
            res.set_header("Pragma", "no-cache");
            res.set_header("Connection", "keep-alive");
            auto last_push = Clock::now();
            res.set_chunked_content_provider(
                "multipart/x-mixed-replace; boundary=frame",
                [this, last_push](size_t, DataSink& sink) mutable {
                    while (cam_running.load()) {
                        std::vector<uint8_t> jpg;
                        {
                            std::unique_lock<std::mutex> lk(frame_mtx);
                            frame_cv.wait_for(lk, std::chrono::milliseconds(1000),
                                              [this]{ return !last_jpeg.empty() || !cam_running.load(); });
                            if (!cam_running.load()) break;
                            jpg = last_jpeg;
                        }
                        // HTTP FPS limit
                        if (http_fps_limit > 0) {
                            auto now = Clock::now();
                            double elapsed = std::chrono::duration<double>(now - last_push).count();
                            double min_dt = 1.0 / http_fps_limit;
                            if (elapsed < min_dt) {
                                auto sleep_d = std::chrono::duration<double>(min_dt - elapsed);
                                std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(sleep_d));
                            }
                            last_push = Clock::now();
                        }

                        std::string header = "--frame\r\n"
                                             "Content-Type: image/jpeg\r\n"
                                             "Content-Length: " + std::to_string(jpg.size()) + "\r\n\r\n";
                        if (!sink.write(header.data(), header.size())) break;
                        if (!sink.write(reinterpret_cast<const char*>(jpg.data()), jpg.size())) break;
                        if (!sink.write("\r\n", 2)) break;
                        std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    }
                    return true;
                },
                [](bool){}
            );
        });

        server.Get("/api/last.json", [this](const Request&, Response& res) {
            json j;
            {
                std::lock_guard<std::mutex> lk(frame_mtx);
                j = last_meta.is_null() ? json::object() : last_meta;
            }
            res.set_content(j.dump(), "application/json");
        });

        server.Get("/api/frame.jpg", [this](const Request&, Response& res) {
            std::vector<uint8_t> jpg;
            {
                std::lock_guard<std::mutex> lk(frame_mtx);
                jpg = last_jpeg;
            }
            if (jpg.empty()) { res.status = 503; res.set_content("no frame", "text/plain"); return; }
            res.set_content(std::string(reinterpret_cast<const char*>(jpg.data()), jpg.size()), "image/jpeg");
        });
    }

    // ---------- Camera (V4L2 MJPEG) ----------
    bool initCamera() {
        cam_fd = open(cam_dev.c_str(), O_RDWR | O_NONBLOCK, 0);
        if (cam_fd < 0) { perror("open camera"); return false; }

        v4l2_format fmt{};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = cam_w_req;
        fmt.fmt.pix.height = cam_h_req;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        fmt.fmt.pix.field = V4L2_FIELD_ANY;
        if (ioctl(cam_fd, VIDIOC_S_FMT, &fmt) < 0) {
            perror("VIDIOC_S_FMT MJPEG");
            close(cam_fd); cam_fd = -1;
            return false;
        }
        cam_w = fmt.fmt.pix.width;
        cam_h = fmt.fmt.pix.height;

        v4l2_streamparm parm{};
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.timeperframe.numerator = 1;
        parm.parm.capture.timeperframe.denominator = cam_fps_req <= 0 ? 30 : cam_fps_req;
        ioctl(cam_fd, VIDIOC_S_PARM, &parm);
        cam_fps = parm.parm.capture.timeperframe.denominator > 0
                  ? parm.parm.capture.timeperframe.denominator : cam_fps_req;

        v4l2_format gfmt{}; gfmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(cam_fd, VIDIOC_G_FMT, &gfmt);
        char fcc[5]; fourcc_to_str(gfmt.fmt.pix.pixelformat, fcc);
        printf("CAP negotiated: %dx%d @ %d FPS, FOURCC=%s\n", cam_w, cam_h, cam_fps, fcc);

        v4l2_requestbuffers req{};
        req.count = std::max(1, cam_buffers);
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if (ioctl(cam_fd, VIDIOC_REQBUFS, &req) < 0 || req.count < 1) {
            perror("VIDIOC_REQBUFS");
            close(cam_fd); cam_fd = -1;
            return false;
        }

        cam_bufs.resize(req.count);
        for (unsigned int i = 0; i < req.count; ++i) {
            v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            if (ioctl(cam_fd, VIDIOC_QUERYBUF, &buf) < 0) { perror("VIDIOC_QUERYBUF"); return false; }
            cam_bufs[i].length = buf.length;
            cam_bufs[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, cam_fd, buf.m.offset);
            if (cam_bufs[i].start == MAP_FAILED) { perror("mmap"); return false; }
        }
        for (unsigned int i = 0; i < req.count; ++i) {
            v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            if (ioctl(cam_fd, VIDIOC_QBUF, &buf) < 0) { perror("VIDIOC_QBUF"); return false; }
        }
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(cam_fd, VIDIOC_STREAMON, &type) < 0) { perror("VIDIOC_STREAMON"); return false; }

        cam_running = true;
        return true;
    }

    void deinitCamera() {
        if (cam_fd >= 0) {
            v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            ioctl(cam_fd, VIDIOC_STREAMOFF, &type);
            for (auto& b : cam_bufs) {
                if (b.start && b.start != MAP_FAILED && b.length) munmap(b.start, b.length);
            }
            cam_bufs.clear();
            close(cam_fd);
            cam_fd = -1;
        }
    }

    bool grabMjpeg(std::vector<uint8_t>& out) {
        if (cam_fd < 0) return false;
        fd_set fds; FD_ZERO(&fds); FD_SET(cam_fd, &fds);
        timeval tv{0}; tv.tv_sec = 2;
        int r = select(cam_fd + 1, &fds, NULL, NULL, &tv);
        if (r <= 0) return false;

        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(cam_fd, VIDIOC_DQBUF, &buf) < 0) return false;
        if (buf.index >= cam_bufs.size()) return false;

        auto& b = cam_bufs[buf.index];
        out.assign((uint8_t*)b.start, (uint8_t*)b.start + buf.bytesused);

        if (ioctl(cam_fd, VIDIOC_QBUF, &buf) < 0) return false;
        return true;
    }

    // ---------- JPEG <-> RGB (TurboJPEG) ----------
    static bool decode_mjpeg_to_image(const uint8_t* jpeg, size_t jpeg_size, image_buffer_t* img) {
        tjhandle th = tjInitDecompress();
        if (!th) return false;
        int w=0, h=0, subsamp=0, colorspace=0;
        if (tjDecompressHeader3(th, jpeg, (unsigned long)jpeg_size, &w, &h, &subsamp, &colorspace) != 0) {
            tjDestroy(th); return false;
        }
        img->width = w; img->height = h;
        img->format = IMAGE_FORMAT_RGB888;
        img->size = w * h * 3;
        img->virt_addr = (unsigned char*)malloc(img->size);
        if (!img->virt_addr) { tjDestroy(th); return false; }

        int rc = tjDecompress2(th, jpeg, (unsigned long)jpeg_size,
                               img->virt_addr, w, 0/*pitch*/, h,
                               TJPF_RGB, TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE);
        tjDestroy(th);
        return rc == 0;
    }

    std::vector<uint8_t> encode_rgb_to_jpeg(const unsigned char* rgb, int w, int h, int q) {
        tjhandle th = tjInitCompress();
        if (!th) return {};
        unsigned char* out = nullptr;
        unsigned long out_sz = 0;
        int rc = tjCompress2(th, rgb, w, 0/*pitch*/, h, TJPF_RGB,
                             &out, &out_sz, TJSAMP_420, q,
                             TJFLAG_FASTDCT);
        std::vector<uint8_t> buf;
        if (rc == 0 && out && out_sz > 0) buf.assign(out, out + out_sz);
        if (out) tjFree(out);
        tjDestroy(th);
        return buf;
    }

    static void freeImage(image_buffer_t& img) {
        if (img.virt_addr) { free(img.virt_addr); img.virt_addr = nullptr; }
    }

    // ---------- main loop ----------
// start main loop (DEBUG profiling) — ВНУТРИ КЛАССА!
    void cameraLoop() {
        static StageAcc acc; // DEBUG: аккумулируем времена по этапам кадра

        auto t_prev = Clock::now();
        double fps_smoothed = 0.0;

        while (cam_running.load()) {
            TICK(loop);  // DEBUG: старт таймера полного цикла

            // ===== CAPTURE =====
            std::vector<uint8_t> mjpeg;
            TICK(cap);  // DEBUG
            bool okCap = grabMjpeg(mjpeg);
            TOCK(cap, acc.cap);  // DEBUG
            if (!okCap) {
                TOCK(loop, acc.loop);        // DEBUG
                acc.add_print_and_reset(30); // DEBUG: печать средних каждые 30 кадров
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            // ===== PREP (MJPEG -> RGB) =====
            TICK(prep);  // DEBUG
            image_buffer_t frame{}; // RGB888
            bool okDec = decode_mjpeg_to_image(mjpeg.data(), mjpeg.size(), &frame);
            TOCK(prep, acc.prep);  // DEBUG
            if (!okDec) {
                TOCK(loop, acc.loop);        // DEBUG
                acc.add_print_and_reset(30); // DEBUG
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // ===== INFER (RKNN) =====
            TICK(infer);  // DEBUG
            object_detect_result_list od{};
            int ret;
            {
                std::lock_guard<std::mutex> lk(infer_mtx);
                ret = inference_yolov8_model(&rknn_app_ctx, &frame, &od);
            }
            TOCK(infer, acc.infer);  // DEBUG

            if (ret == 0) {
                // ===== DRAW =====
                TICK(draw);  // DEBUG
                for (int i = 0; i < od.count; ++i) {
                    auto* d = &od.results[i];
                    int x = d->box.left, y = d->box.top;
                    int w = d->box.right - d->box.left;
                    int h = d->box.bottom - d->box.top;
                    draw_rectangle(&frame, x, y, w, h, COLOR_BLUE, 3);
                    if (show_fps) {
                        char text[96];
                        snprintf(text, sizeof(text), "%s %.1f%%",
                                 coco_cls_to_name(d->cls_id), d->prop * 100.f);
                        draw_text(&frame, text, x, std::max(0, y - 18), COLOR_RED, 10);
                    }
                }
                TOCK(draw, acc.draw);  // DEBUG

                // ===== ENCODE (RGB -> JPEG) =====
                TICK(enc);  // DEBUG
                auto jpg = encode_rgb_to_jpeg(frame.virt_addr, frame.width, frame.height, jpeg_q);
                TOCK(enc, acc.enc);  // DEBUG

                auto meta = formatDetectionResults(&od, frame.width, frame.height);
                {
                    std::lock_guard<std::mutex> lk(frame_mtx);
                    last_jpeg.swap(jpg);
                    last_meta = std::move(meta);
                }
                frame_cv.notify_all();

                // консольный FPS (как было)
                if (show_fps) {
                    auto t_now = Clock::now();
                    double dt = std::chrono::duration<double>(t_now - t_prev).count();
                    t_prev = t_now;
                    double fps_inst = dt > 0 ? 1.0 / dt : 0.0;
                    fps_smoothed = (fps_smoothed == 0.0) ? fps_inst
                                                         : (0.8 * fps_smoothed + 0.2 * fps_inst);
                    static double acc_t = 0; acc_t += dt;
                    if (acc_t >= 1.0) {
                        printf("Loop FPS: %.1f  (cap:%dx%d@%d, model:%dx%d)\n",
                               fps_smoothed, cam_w, cam_h, cam_fps,
                               rknn_app_ctx.model_width, rknn_app_ctx.model_height);
                        acc_t = 0;
                    }
                }
            }

            freeImage(frame);

            TOCK(loop, acc.loop);         // DEBUG: конец таймера цикла
            acc.add_print_and_reset(30);  // DEBUG: печать каждые 30 кадров

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
// end main loop

    static json formatDetectionResults(object_detect_result_list* results, int img_w, int img_h) {
        json detections = json::array();
        for (int i = 0; i < results->count; i++) {
            auto* d = &(results->results[i]);
            json det;
            det["class_name"] = coco_cls_to_name(d->cls_id);
            det["class_id"] = d->cls_id;
            det["confidence"] = d->prop;
            json box;
            box["left"] = d->box.left;
            box["top"] = d->box.top;
            box["right"] = d->box.right;
            box["bottom"] = d->box.bottom;
            box["width"] = d->box.right - d->box.left;
            box["height"] = d->box.bottom - d->box.top;
            det["box"] = box;
            json nbox;
            nbox["left"] = (float)d->box.left / img_w;
            nbox["top"] = (float)d->box.top / img_h;
            nbox["right"] = (float)d->box.right / img_w;
            nbox["bottom"] = (float)d->box.bottom / img_h;
            nbox["width"] = (float)(d->box.right - d->box.left) / img_w;
            nbox["height"] = (float)(d->box.bottom - d->box.top) / img_h;
            det["normalized_box"] = nbox;
            detections.push_back(det);
        }
        json resp;
        resp["detections"] = detections;
        resp["count"] = results->count;
        resp["image_width"] = img_w;
        resp["image_height"] = img_h;
        return resp;
    }
};

// ---- signals & main ----
static YOLOWebServer* g_server = nullptr;
static void signalHandler(int sig) {
    printf("\nSignal %d received, stopping...\n", sig);
    if (g_server) g_server->stop();
}

static void printUsage(const char* argv0){
    printf("Usage: %s <model.rknn> [--dev /dev/videoX] [--port N]\n", argv0);
    printf("  --size WxH           capture size\n");
    printf("  --cap-fps N          capture FPS request\n");
    printf("  --buffers N          V4L2 buffers (1..4)\n");
    printf("  --jpeg-quality N     30..95 (default 70)\n");
    printf("  --http-fps-limit N   limit MJPEG stream FPS (0=unlimited)\n");
    printf("  --fps                print loop FPS to console and draw labels\n");
    printf("  --npu-core auto|0|1|2|01|012  choose NPU core mask\n");
}

int main(int argc, char** argv) {
    if (argc < 2) { printUsage(argv[0]); return -1; }
    Args a = parseArgs(argc, argv);
    if (a.model.empty()) { fprintf(stderr, "Model path is required\n"); return -1; }

    mkdir("./web", 0755);
    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);

    YOLOWebServer app(a);
    g_server = &app;
    if (!app.initialize()) return -1;

    printf("Model: %s\n", a.model.c_str());
    printf("Open stream: http://localhost:%d/api/stream.mjpg\n", a.port);
    app.run();
    return 0;
}

