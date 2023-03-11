#include "opencv_maze_runner.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/optflow.hpp>

#ifdef TARGET_IS_NPS
#include <iostream>
#include <opencv2/highgui.hpp>
#endif

using namespace std;
using namespace cv;
using namespace optflow;

struct optflow_t
{
    Ptr<DenseOpticalFlow> algorithm;
    int h;
    int w;
    Rect roi;
    Mat dist_to_centre;
    Mat gray[2];
    Mat flow_raw;
    Mat flow_uv[2];
    Mat flow_mag;
    Mat flow_ang;
    Mat flow_eof;
    float lmag;
    float rmag;
    float leof;
    float reof;
};

/**
 * declare helper functions, these are functions that help with header functions
 * but since they contain opencv elements, they are declared here
 */
Mat rotate(Mat src);
Mat preprocess(struct image_t *src);

/**
 * global variables, only shared within this file
 */
static struct optflow_t opt_flow;

/**
 * implement functions declared in the header, these can be called in other modules
 */
void opencv_frontend_init(
    uint16_t src_h, uint16_t src_w,
    float src_scale_coef, float of_roi_h_coef, float of_roi_w_coef,
    int of_method)
{
    switch (of_method)
    {
    case FARNEBACK:
        opt_flow.algorithm = createOptFlow_Farneback();
        break;
    case PCAFLOW:
        opt_flow.algorithm = createOptFlow_PCAFlow();
        break;
    case DISMEDIUM:
        opt_flow.algorithm = createOptFlow_DIS(DISOpticalFlow::PRESET_MEDIUM);
        break;
    case DISFAST:
        opt_flow.algorithm = createOptFlow_DIS(DISOpticalFlow::PRESET_FAST);
        break;
    case DISULTRAFAST:
    default:
        opt_flow.algorithm = createOptFlow_DIS(DISOpticalFlow::PRESET_ULTRAFAST);
        break;
    }
    opt_flow.h = (int)(src_h * src_scale_coef);
    opt_flow.w = (int)(src_w * src_scale_coef);
    float roi_h_px = opt_flow.h * of_roi_h_coef;
    float roi_w_px = opt_flow.w * of_roi_w_coef;
    int h_half_px = roi_h_px / 2;
    int w_half_px = roi_w_px / 2;
    opt_flow.roi = Rect(
        opt_flow.w / 2 - w_half_px, opt_flow.h / 2 - h_half_px,
        w_half_px * 2, h_half_px * 2);
    opt_flow.dist_to_centre = Mat::zeros(h_half_px * 2, w_half_px * 2, CV_32FC1);
    for (int i = 0; i < opt_flow.dist_to_centre.rows; i++)
    {
        for (int j = 0; j < opt_flow.dist_to_centre.cols; j++)
        {
            float i_mid = (float)(opt_flow.dist_to_centre.rows - 1) / 2;
            float j_mid = (float)(opt_flow.dist_to_centre.cols - 1) / 2;
            opt_flow.dist_to_centre.at<float>(i, j) = sqrt(pow(i - i_mid, 2) + pow(j - j_mid, 2));
        }
    }
}

void opencv_frontend_run(struct image_t *src_0, struct image_t *src_1)
{
    opt_flow.gray[0] = preprocess(src_0);
    opt_flow.gray[1] = preprocess(src_1);
    opt_flow.algorithm->calc(
        opt_flow.gray[0],
        opt_flow.gray[1],
        opt_flow.flow_raw);
    split(opt_flow.flow_raw, opt_flow.flow_uv);
    multiply(opt_flow.flow_uv[1], -1, opt_flow.flow_uv[1]);
    cartToPolar(
        opt_flow.flow_uv[0], opt_flow.flow_uv[1],
        opt_flow.flow_mag, opt_flow.flow_ang, true);
    divide(opt_flow.flow_mag, opt_flow.dist_to_centre, opt_flow.flow_eof);
    float lmag = 0, rmag = 0, leof = 0, reof = 0;
    int mid = (float)(opt_flow.flow_mag.rows - 1) / 2;
    for (int i = 0; i < opt_flow.flow_mag.rows; i++)
    {
        for (int j = 0; j < opt_flow.flow_mag.cols; j++)
        {
            if (i < mid)
            {
                lmag += opt_flow.flow_mag.at<float>(i, j);
                leof += opt_flow.flow_eof.at<float>(i, j);
            }
            else if (i > mid)
            {
                rmag += opt_flow.flow_mag.at<float>(i, j);
                reof += opt_flow.flow_eof.at<float>(i, j);
            }
        }
    }
    opt_flow.lmag = lmag;
    opt_flow.leof = leof;
    opt_flow.rmag = rmag;
    opt_flow.reof = reof;

#ifdef TARGET_IS_NPS
    Mat src_gray, hsv_split[3], flow_hsv, flow_bgr;

    cvtColor(Mat(src_1->h, src_1->w, CV_8UC2, (char *)src_1->buf), src_gray, CV_YUV2GRAY_Y422);

    normalize(opt_flow.flow_mag, opt_flow.flow_mag, 0, 1, NORM_MINMAX);
    hsv_split[0] = opt_flow.flow_ang;
    hsv_split[1] = Mat::ones(opt_flow.flow_ang.size(), opt_flow.flow_ang.type());
    hsv_split[2] = opt_flow.flow_mag;
    merge(hsv_split, 3, flow_hsv);
    cvtColor(flow_hsv, flow_bgr, COLOR_HSV2BGR);

    imshow("src.gray", rotate(src_gray));
    imshow("optflow.gray[0]", rotate(opt_flow.gray[0]));
    imshow("optflow.gray[1]", rotate(opt_flow.gray[1]));
    imshow("optflow.bgr", rotate(flow_bgr));
    waitKey(1);
#endif
}

struct opencv_frontend_return_t opencv_frontend_return(void)
{
    struct opencv_frontend_return_t ret;
    ret.lmag = opt_flow.lmag;
    ret.rmag = opt_flow.rmag;
    ret.leof = opt_flow.leof;
    ret.reof = opt_flow.reof;
    return ret;
}

#ifdef TARGET_IS_NPS
void opencv_frontend_cbimshow(struct image_t *src)
{
    Mat yuv = Mat(src->h, src->w, CV_8UC2, (char *)src->buf);
    Mat bgr;
    cvtColor(yuv, bgr, CV_YUV2BGR_Y422);
    imshow("cb.bgr", rotate(bgr));
    waitKey(1);
}
#endif

/**
 * implement helper functions
 */
#ifdef TARGET_IS_NPS
Mat rotate(Mat src)
{
    Mat rotated;
    transpose(src, rotated);
    flip(rotated, rotated, 0);
    return rotated;
}
#endif

Mat preprocess(struct image_t *src)
{
    Mat src_gray;
    cvtColor(Mat(src->h, src->w, CV_8UC2, (char *)src->buf), src_gray, CV_YUV2GRAY_Y422);
    resize(src_gray, src_gray, Size(opt_flow.w, opt_flow.h));
    return src_gray(opt_flow.roi).clone();
}