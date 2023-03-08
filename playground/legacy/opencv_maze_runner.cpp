#include "opencv_maze_runner.h"
#include "opencv_image_functions.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;
using namespace optflow;

/*************************************** define the class ***************************************/
class opencv_maze_runner
{
private:

  Ptr<DenseOpticalFlow> optflow_algorithm{};

  int h{}, w{};
  Rect roi_optflow{};
  uint32_t color_cnt{};

  Mat src_prev{}, src{};
  Mat img_prev{}, img{};
  Mat color_ub{}, color_lb{};
  Mat flow{}, color_mask{};


public:

  opencv_maze_runner(){};
  ~opencv_maze_runner(){};
  void set_img_size(int height, int width);
  void set_optflow_params(uint8_t opt, float roi_h_scale, float roi_w_scale);
  void set_color_params(
    uint8_t minlm, uint8_t maxlm,
    uint8_t mincb, uint8_t maxcb,
    uint8_t mincr, uint8_t maxcr
  );
  void handle_input(struct image_t *input);
  void src_to_img();
  void calc_optflow();
  void calc_color(bool draw);
};




/**************************** create global frontend object instance ****************************/
opencv_maze_runner mr_cv_frontend{};




/************************************* implement class func *************************************/
void opencv_maze_runner::set_img_size(int height, int width)
{
  h = height;
  w = width;
  src_prev = Mat::zeros(h, w, CV_8UC2);
  img_prev = Mat::zeros(h, w, CV_8UC2);
  src = Mat::zeros(h, w, CV_8UC2);
  img = Mat::zeros(h, w, CV_8UC2);
  color_ub = Mat::zeros(h, w, CV_8UC2);
  color_lb = Mat::zeros(h, w, CV_8UC2);
}

void opencv_maze_runner::set_optflow_params(uint8_t opt, float roi_h_scale, float roi_w_scale)
{
  switch(opt)
  {
  case FARNEBACK:
    optflow_algorithm = createOptFlow_Farneback();
    break;

  case PCAFLOW:
    optflow_algorithm = createOptFlow_PCAFlow();
    break;

  case DISMEDIUM:
    optflow_algorithm = createOptFlow_DIS(DISOpticalFlow::PRESET_MEDIUM);
    break;

  case DISFAST:
    optflow_algorithm = createOptFlow_DIS(DISOpticalFlow::PRESET_FAST);
    break;

  case DISULTRAFAST:
  default:
    optflow_algorithm = createOptFlow_DIS(DISOpticalFlow::PRESET_ULTRAFAST);
    break;
  }
  
  float roi_h_px = h * roi_h_scale;
  float roi_w_px = w * roi_w_scale;
  int h_offset = roi_h_px / 2;
  int w_offset = roi_w_px / 2;
  roi_optflow = Rect(w / 2 - w_offset, h / 2 - h_offset, w_offset * 2, h_offset * 2);
}

void opencv_maze_runner::set_color_params(
  uint8_t minlm, uint8_t maxlm,
  uint8_t mincb, uint8_t maxcb,
  uint8_t mincr, uint8_t maxcr
)
{
  for(int i = 0; i < h; i++)
  {
    for(int j = 0; j < w; j++)
    {
      if(j % 2 == 0)
      {
        color_lb.at<Vec2b>(i, j).val[0] = mincb;
        color_lb.at<Vec2b>(i, j).val[1] = minlm;
        color_ub.at<Vec2b>(i, j).val[0] = maxcb;
        color_ub.at<Vec2b>(i, j).val[1] = maxlm;
      }
      else
      {
        color_lb.at<Vec2b>(i, j).val[0] = mincr;
        color_lb.at<Vec2b>(i, j).val[1] = minlm;
        color_ub.at<Vec2b>(i, j).val[0] = maxcr;
        color_ub.at<Vec2b>(i, j).val[1] = maxlm;
      }
    }
  }
}

void opencv_maze_runner::handle_input(struct image_t *input)
{
  src_prev = src.clone();
  src = Mat(h, w, CV_8UC2, (char *)input->buf);
}

void opencv_maze_runner::src_to_img()
{
  img_prev = src_prev.clone();
  img = src.clone();
}

void opencv_maze_runner::calc_optflow()
{
  Mat gray, gray_prev;
  cvtColor(img(roi_optflow), gray, CV_YUV2GRAY_Y422);
  cvtColor(img_prev(roi_optflow), gray_prev, CV_YUV2GRAY_Y422);
  optflow_algorithm->calc(gray_prev, gray, flow);

  // visualize
  Mat gray_rotated;
  transpose(gray, gray_rotated);
  flip(gray_rotated, gray_rotated, 0);
  imshow("roi_optflow_gray", gray_rotated);

  Mat mag, ang, flow_uv[2], rgb;
  Mat hsv_split[3], hsv;
  split(flow, flow_uv);
  multiply(flow_uv[1], -1, flow_uv[1]);
  cartToPolar(flow_uv[0], flow_uv[1], mag, ang, true);
  normalize(mag, mag, 0, 1, NORM_MINMAX);
  hsv_split[0] = ang;
  hsv_split[1] = Mat::ones(ang.size(), ang.type());
  hsv_split[2] = mag;
  merge(hsv_split, 3, hsv);
  cvtColor(hsv, rgb, COLOR_HSV2BGR);

  Mat rgb_rotated;
  transpose(rgb, rgb_rotated);
  flip(rgb_rotated, rgb_rotated, 0);
  imshow("flow", rgb_rotated);
  waitKey(1);
}

void opencv_maze_runner::calc_color(bool draw)
{
  inRange(img, color_lb, color_ub, color_mask);
  color_cnt = countNonZero(color_mask);
}

/******************************************* wrappers *******************************************/
void cv_set_img_size(int h, int w)
{
  mr_cv_frontend.set_img_size(h, w);
}

void cv_set_optflow_params(uint8_t opt, float roi_h_scale, float roi_w_scale)
{
  mr_cv_frontend.set_optflow_params(opt, roi_h_scale, roi_w_scale);
}

void cv_set_color_params(
  uint8_t minlm, uint8_t maxlm,
  uint8_t mincb, uint8_t maxcb,
  uint8_t mincr, uint8_t maxcr
)
{
  mr_cv_frontend.set_color_params
  (
    minlm, maxlm,
    mincb, maxcb,
    mincr, maxcr
  );
}

void cv_handle_input(struct image_t *input)
{
  mr_cv_frontend.handle_input(input);
}

void cv_src_to_img()
{
  mr_cv_frontend.src_to_img();
}

void cv_calc_optflow()
{
  mr_cv_frontend.calc_optflow();
}

void cv_calc_color(bool draw)
{
  mr_cv_frontend.calc_color(draw);
}
