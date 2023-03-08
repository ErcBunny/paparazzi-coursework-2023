#include "modules/computer_vision/cv_maze_runner.h"

#define PRINT(string, ...) fprintf(stderr, "[cv->%s()] " string, __FUNCTION__, ##__VA_ARGS__)
#if CV_MAZE_RUNNER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;
static bool first_cb = true;
static bool cv_initialized = false;
static bool run_cv_proc;

struct image_t *video_cb(struct image_t *img, uint8_t camera_id)
{
  if(first_cb)
  {
    cv_set_img_size(img->h, img->w);
    cv_set_optflow_params(FARNEBACK, 1, 1);
    cv_set_color_params(
      LM_MIN, LM_MAX,
      CB_MIN, CB_MAX,
      CR_MIN, CR_MAX
    );
    
    pthread_mutex_lock(&mutex);
    cv_initialized = true;
    pthread_mutex_unlock(&mutex);
    first_cb = false;
  }
  else
  {
    pthread_mutex_lock(&mutex);
    cv_handle_input(img);
    cv_src_to_img();
    cv_calc_optflow();  
    pthread_mutex_unlock(&mutex);
  }
  return NULL;
}

void cv_maze_runner_init(void)
{
  // register callback: camera_id is set by user
  // using front or down cam is set in cv_maze_runner.xml and bebop_course_maze_runner.xml
  cv_add_to_device(&CAMERA, video_cb, MAX_FPS, 0);
  return;
}

void cv_maze_runner_loop(void)
{

#if SIM
  struct timeval start_time, end_time;
  long elapsed_time;
  gettimeofday(&start_time, NULL);
#endif

  // // use mutex when accessing shared resources
  // pthread_mutex_lock(&mutex);
  // if(cv_initialized)
  // {
  //   run_cv_proc = true;
  //   cv_src_to_img();
  // }
  // pthread_mutex_unlock(&mutex);
  // // put heavy stuff out of mutex lock to prevent blocking the queue
  // if(run_cv_proc)
  // {
  //   cv_calc_optflow();
  //   // cv_calc_color(true);
  // }

#if SIM
  gettimeofday(&end_time, NULL);
  elapsed_time = (end_time.tv_sec - start_time.tv_sec) * 1000000 +
                  (end_time.tv_usec - start_time.tv_usec);
  // VERBOSE_PRINT("%f hz\n", (float)1e6/elapsed_time);
#endif

  return;
}