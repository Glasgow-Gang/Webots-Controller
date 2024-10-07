#include "main.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"

#include <cstdio>

/* Utility function */
double clamp(double value, double min, double max) {
  return (value < min) ? min : (value > max) ? max : value;
}

int main() {
  NaoRobot nao;
  while (nao.step(nao.timeStep) != -1) {
    printf("L: %f, R: %f\n",
           nao.GetDistanceSensorValue(NaoRobot::DistanceSensorID::LEFT),
           nao.GetDistanceSensorValue(NaoRobot::DistanceSensorID::RIGHT));
    nao.BodyPositionControl(NaoRobot::ActuatorID::LShoulderPitch, INFINITY);
    nao.BodyVelocityControl(NaoRobot::ActuatorID::LShoulderPitch, 0.8);
    nao.BodyTorqueControl(NaoRobot::ActuatorID::RShoulderPitch, 0.1);

    nao.BodyPositionControl(
        NaoRobot::ActuatorID::HeadYaw,
        static_cast<int>(nao.getTime() * 2) % 5 * 0.2 - 0.5);
  }
  return 0;
}
