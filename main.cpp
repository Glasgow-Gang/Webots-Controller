#include "main.hpp"

#include "left_arm.hpp"
#include "left_leg.hpp"
#include "right_arm.hpp"
#include "right_leg.hpp"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

#include <webots/utils/Motion.hpp>

#define G 9.81

#define TIME_RATE 0.1

NaoRobot *NaoRobot::nao_robot = nullptr;
bool NaoRobot::control_enable = true;
LeftArm *left_arm = nullptr;
RightArm *right_arm = nullptr;
LeftLeg *left_leg = nullptr;
RightLeg *right_leg = nullptr;

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

  left_arm = new LeftArm();
  right_arm = new RightArm();
  left_leg = new LeftLeg();
  right_leg = new RightLeg();

  while (1) {
    LibXR::Thread::Sleep(1000);
  }

  return 0;
}
