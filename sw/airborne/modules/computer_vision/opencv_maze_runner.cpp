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

Mat prevgray;
Ptr<DenseOpticalFlow> algorithm = createOptFlow_DIS(DISOpticalFlow::PRESET_MEDIUM);

void opencv_frontend_run(struct image_t *input)
{
  Mat src_yuv, src_bgr, src_gray;
  src_yuv = Mat(input->h, input->w, CV_8UC2, (char *)input->buf);
  cvtColor(src_yuv, src_gray, CV_YUV2GRAY_Y422);

  Mat rgb, frame;
  Mat flow, flow_uv[2];
  Mat mag, ang;
  Mat hsv_split[3], hsv;
  if (!prevgray.empty())
  {
    algorithm->calc(prevgray, src_gray, flow);
    split(flow, flow_uv);
    multiply(flow_uv[1], -1, flow_uv[1]);
    cartToPolar(flow_uv[0], flow_uv[1], mag, ang, true);
    normalize(mag, mag, 0, 1, NORM_MINMAX);
    hsv_split[0] = ang;
    hsv_split[1] = Mat::ones(ang.size(), ang.type());
    hsv_split[2] = mag;
    merge(hsv_split, 3, hsv);
    cvtColor(hsv, rgb, COLOR_HSV2BGR);
    imshow("flow", rotate(rgb));
    imshow("orig", rotate(src_gray));
  }
  std::swap(prevgray, src_gray);

  waitKey(1);
}