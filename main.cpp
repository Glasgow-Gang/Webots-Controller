#include "main.hpp"

#include "left_arm.hpp"
#include "right_arm.hpp"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

#define G 9.81

#define TIME_RATE 0.1

NaoRobot *NaoRobot::nao_robot = nullptr;

/* Utility function */
double clamp(double value, double min, double max) {
  return (value < min) ? min : (value > max) ? max : value;
}

int main() {
  auto console = spdlog::stdout_color_mt("console");

  spdlog::set_level(spdlog::level::trace);

  LibXR::PlatformInit();

  /* Wait for initizalization */
  LibXR::Thread::Sleep(1);

  NaoRobot nao;

  LeftArm left_arm;
  RightArm right_arm;

  while (1) {
    LibXR::Thread::Sleep(1);
    continue;
    static int count = 0, fsm = 0;

    count++;
    if (count > 1000) {
      count = 0;
      switch (fsm) {
      case 0:
        nao.target_position_[static_cast<int>(NaoRobot::JointID::HeadPitch)] =
            0.5;
        nao.target_position_[static_cast<int>(NaoRobot::JointID::HeadYaw)] =
            0.5;
        fsm = 1;
        break;
      case 1:
        nao.target_position_[static_cast<int>(NaoRobot::JointID::HeadPitch)] =
            -0.5;
        nao.target_position_[static_cast<int>(NaoRobot::JointID::HeadYaw)] =
            -0.5;
        fsm = 2;
        break;
      case 2:
        nao.target_position_[static_cast<int>(NaoRobot::JointID::HeadPitch)] =
            0.5;
        nao.target_position_[static_cast<int>(NaoRobot::JointID::HeadYaw)] =
            -0.5;
        fsm = 3;
        break;
      case 3:
        nao.target_position_[static_cast<int>(NaoRobot::JointID::HeadPitch)] =
            -0.5;
        nao.target_position_[static_cast<int>(NaoRobot::JointID::HeadYaw)] =
            0.5;
        fsm = 0;
        break;
      }
    }

    nao.UpdateEulr();
    nao.HeadControl();
  }
  return 0;
}
