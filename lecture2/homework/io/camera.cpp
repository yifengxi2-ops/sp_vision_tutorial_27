#include "io/camera.hpp"
#include "hikrobot/include/MvCameraControl.h"

Camera::Camera()
{

  MV_CC_DEVICE_INFO_LIST device_list;
  int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    return;
  }
  if (device_list.nDeviceNum == 0) {
    return;
  }

  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
  if (ret != MV_OK) {
    return;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    return;
  }


  MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
  MV_CC_SetFloatValue(handle_, "ExposureTime", 5000);
  MV_CC_SetFloatValue(handle_, "Gain", 10);
  MV_CC_SetFrameRate(handle_, 60);


  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    return;
  }
  grabbing_ = true;
}

Camera::~Camera()
{
  if (handle_ == nullptr) return;

  if (grabbing_) MV_CC_StopGrabbing(handle_);
  MV_CC_CloseDevice(handle_);
  MV_CC_DestroyHandle(handle_);

  handle_ = nullptr;
  grabbing_ = false;
}

cv::Mat Camera::read()
{
  if (handle_ == nullptr || !grabbing_) return {};

  MV_FRAME_OUT raw;
  unsigned int nMsec = 100;
  int ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
  if (ret != MV_OK) {
    return {};
  }

  const auto & info = raw.stFrameInfo;

  MV_CC_PIXEL_CONVERT_PARAM cvt_param;
  cv::Mat img(cv::Size(info.nWidth, info.nHeight), CV_8U, raw.pBufAddr);

  cvt_param.nWidth = info.nWidth;
  cvt_param.nHeight = info.nHeight;
  cvt_param.pSrcData = raw.pBufAddr;
  cvt_param.nSrcDataLen = info.nFrameLen;
  cvt_param.enSrcPixelType = info.enPixelType;
  cvt_param.pDstBuffer = img.data;
  cvt_param.nDstBufferSize = img.total() * img.elemSize();
  cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

  const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
    {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
    {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
    {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
    {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};
  cv::cvtColor(img, img, type_map.at(info.enPixelType));

  cv::Mat bgr = img.clone();
  MV_CC_FreeImageBuffer(handle_, &raw);

  return bgr;
}