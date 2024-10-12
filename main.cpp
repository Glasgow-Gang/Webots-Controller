#include "main.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cstdio>

#include "inertial.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "params.hpp"

#define G 9.81

/* Utility function */
double clamp(double value, double min, double max) {
  return (value < min) ? min : (value > max) ? max : value;
}

int main() {
  NaoRobot nao;

  while (nao.step(nao.timeStep) != -1) {
    static int count = 0, fsm = 0;

    count++;
    if (count > 500) {
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
