#include "main.hpp"
#include "sdl.hpp"
#include "thread.hpp"

#include <unistd.h>
#include <webots/utils/Motion.hpp>

#include <SDL.h>
#include <SDL_image.h>

#define G 9.81

#define TIME_RATE 0.1

NaoRobot *NaoRobot::nao_robot = nullptr;
Sim2D *Sim2D::sim2d = nullptr;

/* Utility function */
double clamp(double value, double min, double max) {
  return (value < min) ? min : (value > max) ? max : value;
}

int main() {
  NaoRobot nao_robot;

  LibXR::PlatformInit(&nao_robot.supervisor);

  Sim2D sim2d;

  while (true) {
    nao_robot.RobotMoveAround(nao_robot.ball_pos.x(), nao_robot.ball_pos.y(),
                              0.05, nao_robot.BallGetAngleToEnemyGate());
    LibXR::Thread::Sleep(1);
  }

  return 0;
}
