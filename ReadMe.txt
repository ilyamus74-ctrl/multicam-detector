# стандартно (video0, 640x480@30, порт 8080)
./yolov8_web_server ./yolov8.rknn --fps

# другая камера и размер
./yolov8_web_server ./yolov8.rknn --dev /dev/video1 --size 1280x720 --cap-fps 30 --port 9090 --fps

Про разрешение и FPS

    Модель берёт входной размер из rknn (model_width/height, обычно 640×640). Наш препроцесс внутри inference_yolov8_model сам сделает letterbox/resize, так что --size (захват) можно ставить 640×480, 1280×720 — без пересборки модели.

    --cap-fps — это запрос драйверу камеры; реальный FPS выводим в OSD (--fps) и в /api/health (cap_real_fps).
