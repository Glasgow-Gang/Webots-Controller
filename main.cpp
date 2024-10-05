#include "main.hpp"

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
  }
  return 0;
}
