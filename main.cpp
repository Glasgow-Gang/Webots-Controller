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

  bool ans = true;
  int fsm = 0;

  while (true) {

    if (!nao_robot.BallInOurField()) {
      fsm = 0;
      double x = 0.40, y = 0.28;
      ans = nao_robot.RobotGoto(x, y, 0.1);
      if (!ans) {
        nao_robot.RobotTurn(0);
      }
      continue;
    }

    switch (fsm) {
    case 0:
      if (ans == 0) {
        fsm++;
        break;
      }
      ans = nao_robot.RobotMoveAround(
          nao_robot.ball_pos.x(), nao_robot.ball_pos.y(), 0.15,
          nao_robot.BallGetAngleToOwnGate(), 2, 0.04);
      break;
    case 1:

      nao_robot.RobotGoto(nao_robot.ball_pos.x(), nao_robot.ball_pos.y());
      if (nao_robot.RobotGetDistanceTo(nao_robot.ball_pos.x(),
                                       nao_robot.ball_pos.y()) < 0.025) {
        fsm++;
      } else {
        printf("distance: %f\n",
               nao_robot.RobotGetDistanceTo(nao_robot.ball_pos.x(),
                                            nao_robot.ball_pos.y()));
      }
      break;
    case 2:
      while (!nao_robot.moving)
        ;
      LibXR::Thread::Sleep(100);
      nao_robot.PlayMotion(LongPass);
      LibXR::Thread::Sleep(100);
      fsm = 0;
      break;
    }
    LibXR::Thread::Sleep(10);
  }
  return 0;
}
