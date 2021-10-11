#include "wolfvision.hpp"

int main() {
  fmt::print("[{}] WolfVision built on g++ version: {}\n", idntifier, __VERSION__);
  fmt::print("[{}] WolfVision opencv version: {}\n", idntifier, CV_VERSION);
  fmt::print("[{}] WolfVision config file path: {}\n", idntifier, CONFIG_FILE_PATH);

  cv::Mat src_img_, roi_img_;

  mindvision::VideoCapture* mv_capture_ = new mindvision::VideoCapture(
    mindvision::CameraParam(0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_600));

  uart::SerialPort serial_ = uart::SerialPort(
    fmt::format("{}{}", CONFIG_FILE_PATH, "/serial/uart_serial_config.xml"));

  cv::VideoCapture cap_ = cv::VideoCapture("/home/dji/workspace_vscode/rm_2021_-code/Vedio_Record/403.avi");

  basic_armor::Detector basic_armor_ = basic_armor::Detector(
    fmt::format("{}{}", CONFIG_FILE_PATH, "/armor/basic_armor_config.xml"));

  basic_pnp::PnP pnp_ = basic_pnp::PnP(
    fmt::format("{}{}", CONFIG_FILE_PATH, "/camera/cameraParams_555.xml"),
    fmt::format("{}{}", CONFIG_FILE_PATH, "/angle_solve/basic_pnp_config.xml"));

  fps::FPS       global_fps_;

  size_t         serial_cnt_{ 0 };

  serial_.updateReceiveInformation();
  int num = 0;
  int temp = 0;
  while (true) {
    global_fps_.getTick();
    if (mv_capture_->isindustryimgInput()) {
      src_img_ = mv_capture_->image();
    } else {
      cap_.read(src_img_);
    }

    // if (!src_img_.empty()) {
    //   if (++serial_cnt_ % 300 == 0) { // 每循环 300 次更新一次串口数据

    //   }
    // }

    serial_.updateReceiveInformation();
    if (basic_armor_.runBasicArmor(src_img_, serial_.returnReceive())) {
      pnp_.solvePnP(serial_.returnReceiveBulletVelocity(), basic_armor_.returnFinalArmorDistinguish(0), basic_armor_.returnFinalArmorRotatedRect(0));
      num++;
      if ( num < 500) {
        serial_.updataWriteData(pnp_.returnYawAngle(), pnp_.returnPitchAngle(), pnp_.returnDepth(), basic_armor_.returnArmorNum(), 0);
      } else {
        temp++;
        if(temp < 70) {
          serial_.updataWriteData(pnp_.returnYawAngle(), pnp_.returnPitchAngle(), pnp_.returnDepth(), 0, 0);
        } else {
          temp = 0;
          num  = 0;
        }
      }
    } else {
      serial_.updataWriteData(pnp_.returnYawAngle(), pnp_.returnPitchAngle(), pnp_.returnDepth(), 0, 0);
    }
    
    // 非击打哨兵模式时初始化
    if (serial_.returnReceiveMode() != uart::SENTRY_STRIKE_MODE) {
      basic_armor_.initializationSentryMode();
    }

    mv_capture_->cameraReleasebuff();
    basic_armor_.freeMemory();
#ifndef RELEASE
    cv::imshow("dafule", src_img_);
    if (cv::waitKey(1) == 'q') {
      return 0;
    }
#else
    usleep(1);
#endif
    global_fps_.calculateFPSGlobal();
   if (global_fps_.returnFps() > 500 || global_fps_.returnFps() < 1) {
      mv_capture_->~VideoCapture();
      static int counter_for_dev { 100 };
      static int counter_for_new { 30 };
      while (!utils::resetMVCamera()) {
        if (!--counter_for_dev) {
          int i [[maybe_unused]] = std::system("echo dji | sudo -S reboot");
        }
        usleep(100);
      }
      usleep(100);
      mv_capture_ = new mindvision::VideoCapture(mindvision::CameraParam(
          0, mindvision::RESOLUTION_1280_X_800, mindvision::EXPOSURE_600));
      if (!--counter_for_new) {
        int i [[maybe_unused]] = std::system("echo dji | sudo -S reboot");
      }
    } 
  }
  return 0;
}
