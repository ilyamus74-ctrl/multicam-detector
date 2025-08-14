// utils/v4l2_capture.h
#ifndef V4L2_CAPTURE_H
#define V4L2_CAPTURE_H

#include <string>

class V4L2_Capture {
public:
    V4L2_Capture();
    ~V4L2_Capture();

    int Open(const std::string& devName, int width, int height);
    void Close();
    char* GetFrame();
    void ReleaseFrame();
    int GetFrameWidth();
    int GetFrameHeight();
    int GetFrameStride();
    int GetFrameFormat();
private:
    int fd;
    int width;
    int height;
    int stride;
    int format;
    int buffer_count;
    struct buffer {
        void* start;
        size_t length;
    };
    buffer* buffers;
    size_t frame_size;
    void* current_frame_buffer;
};

#endif // V4L2_CAPTURE_H
