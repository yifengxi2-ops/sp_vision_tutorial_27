#ifndef IO_CAMERA_HPP
#define IO_CAMERA_HPP
#include <opencv2/opencv.hpp>

class Camera
{
public:
  Camera();            // 打开设备+设置参数+开始采集
  ~Camera();           // 停止采集+关设备+销毁句柄
  cv::Mat read();      // 取帧

private:
  void * handle_ = nullptr;
  bool grabbing_ = false;
};

#endif