#ifndef _RKNN_YOLOV8_POSTPROCESS_H_
#define _RKNN_YOLOV8_POSTPROCESS_H_

#include <vector>
#include "rknn_api.h"
#include "common.h"
#include "postprocess.h"

// Эта функция должна быть объявлена
int post_process_yolov8(rknn_context rknn_ctx,
                        rknn_output *outputs,
                        int model_height, int model_width,
                        float conf_threshold, float nms_threshold,
                        BOX_RECT pads,
                        int original_width, int original_height,
                        detect_result_group_t *group);

#endif
