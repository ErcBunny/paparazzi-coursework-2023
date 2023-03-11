#ifndef OPENCV_MAZE_RUNNER_H
#define OPENCV_MAZE_RUNNER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"

#define FARNEBACK 0
#define PCAFLOW 1
#define DISMEDIUM 2
#define DISFAST 3
#define DISULTRAFAST 4

    struct opencv_frontend_return_t
    {
        float lmag;
        float rmag;
        float leof;
        float reof;
    };

    void opencv_frontend_init(
        uint16_t src_h, uint16_t src_w,
        float src_scale_coef, float of_roi_h_coef, float of_roi_w_coef,
        int of_method);

    void opencv_frontend_run(struct image_t *src_0, struct image_t *src_1);

    struct opencv_frontend_return_t opencv_frontend_return(void);

    void opencv_frontend_cbimshow(struct image_t *src);

#ifdef __cplusplus
}
#endif

#endif