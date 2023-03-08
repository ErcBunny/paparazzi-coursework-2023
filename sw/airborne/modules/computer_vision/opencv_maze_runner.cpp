#include "opencv_maze_runner.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/highgui.hpp>


using namespace std;
using namespace cv;
using namespace optflow;

Mat rotate(Mat src);
Mat rotate(Mat src)
{
  Mat rotated;
  transpose(src, rotated);
  flip(rotated, rotated, 0);
  return rotated;
}

void opencv_frontend_run(
  struct image_t *input, int optflow_algo, float roi_flow_h_scale, float roi_flow_w_scale, bool draw, bool opencvshow,
  uint8_t lum_min, uint8_t lum_max,
  uint8_t cb_min, uint8_t cb_max,
  uint8_t cr_min, uint8_t cr_max,
  uint32_t *color_count
)
{
  static bool first_run = true;
  static Mat prev_gray;
  static Ptr<DenseOpticalFlow> algorithm;
  static Rect roi_optflow;

  Mat src_gray;
  Mat rgb, frame;
  Mat flow, flow_uv[2];
  Mat mag, ang;
  Mat hsv_split[3], hsv;
  cvtColor(Mat(input->h, input->w, CV_8UC2, (char *)input->buf), src_gray, CV_YUV2GRAY_Y422);

  if(first_run)
  {
    prev_gray = Mat::zeros(input->h, input->w, CV_8UC1);
    rgb = Mat::zeros(input->h, input->w, CV_8UC3);
    switch(optflow_algo)
    {
    case FARNEBACK:
      algorithm = createOptFlow_Farneback();
      break;

    case PCAFLOW:
      algorithm = createOptFlow_PCAFlow();
      break;

    case DISMEDIUM:
      algorithm = createOptFlow_DIS(DISOpticalFlow::PRESET_MEDIUM);
      break;

    case DISFAST:
      algorithm = createOptFlow_DIS(DISOpticalFlow::PRESET_FAST);
      break;

    case DISULTRAFAST:
    default:
      algorithm = createOptFlow_DIS(DISOpticalFlow::PRESET_ULTRAFAST);
      break;
    }
    float roi_h_px = input->h * roi_flow_h_scale;
    float roi_w_px = input->w * roi_flow_w_scale;
    int h_offset = roi_h_px / 2;
    int w_offset = roi_w_px / 2;
    roi_optflow = Rect(input->w / 2 - w_offset, input->h / 2 - h_offset, w_offset * 2, h_offset * 2);
    // it seems that using roi to crop image is a bit slow
    first_run = false;
  }
  else
  {
    // optical flow
    algorithm->calc(prev_gray, src_gray, flow);
    split(flow, flow_uv);
    multiply(flow_uv[1], -1, flow_uv[1]);
    cartToPolar(flow_uv[0], flow_uv[1], mag, ang, true);
    normalize(mag, mag, 0, 1, NORM_MINMAX);
    hsv_split[0] = ang;
    hsv_split[1] = Mat::ones(ang.size(), ang.type());
    hsv_split[2] = mag;
    merge(hsv_split, 3, hsv);
    cvtColor(hsv, rgb, COLOR_HSV2BGR);
  }
  std::swap(prev_gray, src_gray);

  //color detection
  Mat color_mask = Mat::zeros(input->h, input->w, CV_8UC1);
  uint32_t cnt = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = (uint8_t *)input->buf;

  for (uint16_t y = 0; y < input->h; y++) {
    for (uint16_t x = 0; x < input->w; x ++) {
      // Check if the color is inside the specified values
      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * input->w + 2 * x];      // U
        yp = &buffer[y * 2 * input->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * input->w + 2 * x + 2];  // V
        //yp = &buffer[y * 2 * input->w + 2 * x + 3]; // Y2
      } else {
        // Uneven x
        up = &buffer[y * 2 * input->w + 2 * x - 2];  // U
        //yp = &buffer[y * 2 * input->w + 2 * x - 1]; // Y1
        vp = &buffer[y * 2 * input->w + 2 * x];      // V
        yp = &buffer[y * 2 * input->w + 2 * x + 1];  // Y2
      }
      if ( (*yp >= lum_min) && (*yp <= lum_max) &&
           (*up >= cb_min ) && (*up <= cb_max ) &&
           (*vp >= cr_min ) && (*vp <= cr_max )) {
        cnt ++;
        tot_x += x;
        tot_y += y;
        if (draw){
          *yp = 255;  // make pixel brighter in image
          color_mask.at<uchar>(y, x) = 255;
        }
      }
    }
  }
  *color_count = cnt;

  if(opencvshow)
  {
    imshow("orig", rotate(src_gray));
    imshow("flow", rotate(rgb));
    imshow("color", rotate(color_mask));

    waitKey(1);
  }
}
