#include "main.hpp"
#include "sdl.hpp"
#include "thread.hpp"

#include <cstring>
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

int main(int argc, char **argv) {

  RobotType robot_type = Defender1;
  double gate_addr = GATE_1_X;

  if (argc > 2) {
    if (strcmp(argv[1], "defender1") == 0) {
      robot_type = Defender1;
    } else if (strcmp(argv[1], "defender2") == 0) {
      robot_type = Defender2;
    } else if (strcmp(argv[1], "attacker") == 0) {
      robot_type = Attacker;
    } else if (strcmp(argv[1], "goalkeeper") == 0) {
      robot_type = GoalKeeper;
    }

    if (strcmp(argv[2], "1") == 0) {
      gate_addr = GATE_1_X;
    } else if (strcmp(argv[2], "2") == 0) {
      gate_addr = GATE_2_X;
    }
  }

  NaoRobot nao_robot(robot_type, gate_addr);

  LibXR::PlatformInit(&nao_robot.supervisor);

  int index = robot_type;

  if (gate_addr == GATE_2_X) {
    index += 4;
  }

  Sim2D sim2d(index);

  bool ans = true;
  int fsm = 0;

  if (robot_type == Defender1) {
    while (true) {

      std::cout << "fsm: " << fsm << std::endl;

      if (!nao_robot.BallInOurField() || !nao_robot.BallInDefender1Field()) {
        fsm = 0;
        ans = nao_robot.RobotGoto(nao_robot.GetOwnDefenderInitX(),
                                  nao_robot.GetOwnDefenderInitY(), 0.15);
        if (!ans) {
          nao_robot.RobotTurn(0);
        }
        continue;
      }

      switch (fsm) {
      case 0:
        if (ans == 0) {
          fsm++;
          ans = true;
          break;
        }
        ans = nao_robot.RobotMoveAround(
            nao_robot.ball_pos.x(), nao_robot.ball_pos.y(), 0.15,
            nao_robot.BallGetAngleToEnemyGate(), 2, 0.04);
        break;
      case 1:

        nao_robot.RobotGoto(nao_robot.ball_pos.x(), nao_robot.ball_pos.y());
        if (nao_robot.RobotGetDistanceTo(nao_robot.ball_pos.x(),
                                         nao_robot.ball_pos.y()) < 0.03) {
          fsm++;
        } else {
          printf("distance: %f\n",
                 nao_robot.RobotGetDistanceTo(nao_robot.ball_pos.x(),
                                              nao_robot.ball_pos.y()));
        }
        break;
      case 2:
        LibXR::Thread::Sleep(100);
        nao_robot.PlayMotion(LongPass);
        std::cout << "LongPass\n";

        for (int i = 0; i < 6000; i++) {
          if (nao_robot.PlayMotion(LongPass)) {
            break;
          }

          LibXR::Thread::Sleep(2);
        }
        LibXR::Thread::Sleep(100);

        for (int i = 0; i < 6000; i++) {
          if (!nao_robot.moving) {
            break;
          }

          LibXR::Thread::Sleep(2);
        }

        LibXR::Thread::Sleep(100);
        nao_robot.PlayMotion(LongPass);
        std::cout << "LongPass\n";

        for (int i = 0; i < 6000; i++) {
          if (nao_robot.PlayMotion(LongPass)) {
            break;
          }

          LibXR::Thread::Sleep(2);
        }
        LibXR::Thread::Sleep(100);

        for (int i = 0; i < 6000; i++) {
          if (!nao_robot.moving) {
            break;
          }

          LibXR::Thread::Sleep(2);
        }

        if (nao_robot.BallInOurField()) {
          ans = 1;
          fsm++;
        } else {
          fsm = 0;
        }
        break;
      case 3:
        if (ans == 0) {
          ans = true;
          fsm = 1;
          break;
        }
        ans = nao_robot.RobotMoveAround(
            nao_robot.ball_pos.x(), nao_robot.ball_pos.y(), 0.07,
            nao_robot.BallGetAngleToEnemyGate(), 2, 0.02);
        break;
      default:
        fsm = 0;
        break;
      }
      LibXR::Thread::Sleep(20);
    }
  } else if (robot_type == Defender2) {
    while (true) {

      std::cout << "fsm: " << fsm << std::endl;

      if (!nao_robot.BallInOurField() || nao_robot.BallInDefender1Field()) {
        fsm = 0;
        ans = nao_robot.RobotGoto(nao_robot.GetOwnDefenderInitX(),
                                  1.0 - nao_robot.GetOwnDefenderInitY(), 0.15);
        if (!ans) {
          nao_robot.RobotTurn(0);
        }
        continue;
      }

      switch (fsm) {
      case 0:
        if (ans == 0) {
          fsm++;
          ans = true;
          break;
        }
        ans = nao_robot.RobotMoveAround(
            nao_robot.ball_pos.x(), nao_robot.ball_pos.y(), 0.15,
            nao_robot.BallGetAngleToEnemyGate(), 2, 0.04);
        break;
      case 1:

        nao_robot.RobotGoto(nao_robot.ball_pos.x(), nao_robot.ball_pos.y());
        if (nao_robot.RobotGetDistanceTo(nao_robot.ball_pos.x(),
                                         nao_robot.ball_pos.y()) < 0.03) {
          fsm++;
        } else {
          printf("distance: %f\n",
                 nao_robot.RobotGetDistanceTo(nao_robot.ball_pos.x(),
                                              nao_robot.ball_pos.y()));
        }
        break;
      case 2:
        LibXR::Thread::Sleep(100);
        nao_robot.PlayMotion(LongPass);
        std::cout << "LongPass\n";

        for (int i = 0; i < 6000; i++) {
          if (nao_robot.PlayMotion(LongPass)) {
            break;
          }

          LibXR::Thread::Sleep(2);
        }
        LibXR::Thread::Sleep(100);

        for (int i = 0; i < 6000; i++) {
          if (!nao_robot.moving) {
            break;
          }

          LibXR::Thread::Sleep(2);
        }

        LibXR::Thread::Sleep(100);
        nao_robot.PlayMotion(LongPass);
        std::cout << "LongPass\n";

        for (int i = 0; i < 6000; i++) {
          if (nao_robot.PlayMotion(LongPass)) {
            break;
          }

          LibXR::Thread::Sleep(2);
        }
        LibXR::Thread::Sleep(100);

        for (int i = 0; i < 6000; i++) {
          if (!nao_robot.moving) {
            break;
          }

          LibXR::Thread::Sleep(2);
        }

        if (nao_robot.BallInOurField()) {
          ans = 1;
          fsm++;
        } else {
          fsm = 0;
        }
        break;
      case 3:
        if (ans == 0) {
          ans = true;
          fsm = 1;
          break;
        }
        ans = nao_robot.RobotMoveAround(
            nao_robot.ball_pos.x(), nao_robot.ball_pos.y(), 0.07,
            nao_robot.BallGetAngleToEnemyGate(), 2, 0.02);
        break;
      default:
        fsm = 0;
        break;
      }
      LibXR::Thread::Sleep(20);
    }
  } else if (robot_type == Attacker) {
    ans = 1;
    while (true) {
      std::cout << "fsm: " << fsm << std::endl;

      if (nao_robot.BallInOurField()) {
        fsm = 0;
        ans = nao_robot.RobotGoto(nao_robot.GetOwnAttackerInitX(),
                                  nao_robot.GetOwnAttackerInitY(), 0.15);
        if (!ans) {
          ans = true;
          nao_robot.RobotTurn(0);
        }
        continue;
      }

      switch (fsm) {
      case 0:
        if (ans == 0) {
          fsm++;
          ans = true;
          break;
        }
        ans = nao_robot.RobotMoveAround(
            nao_robot.ball_pos.x(), nao_robot.ball_pos.y(), 0.1,
            nao_robot.BallGetAngleToEnemyGate(), 3, 0.05);
        break;
      case 1:
        nao_robot.RobotGoto(nao_robot.ball_pos.x(), nao_robot.ball_pos.y());
        if (nao_robot.RobotGetDistanceTo(nao_robot.ball_pos.x(),
                                         nao_robot.ball_pos.y()) < 0.02) {
          fsm++;
        } else {
          printf("distance: %f\n",
                 nao_robot.RobotGetDistanceTo(nao_robot.ball_pos.x(),
                                              nao_robot.ball_pos.y()));
        }
        break;
      case 2:
        LibXR::Thread::Sleep(100);
        nao_robot.PlayMotion(LongPass);
        std::cout << "Kick\n";

        for (int i = 0; i < 6000; i++) {
          if (nao_robot.PlayMotion(LongPass)) {
            break;
          }

          LibXR::Thread::Sleep(2);
        }
        LibXR::Thread::Sleep(100);

        for (int i = 0; i < 6000; i++) {
          if (!nao_robot.moving) {
            break;
          }

          LibXR::Thread::Sleep(2);
        }

        if (!nao_robot.BallInOurField()) {
          ans = 1;
          fsm++;
        } else {
          fsm = 0;
        }
        break;
      case 3:
        if (ans == 0) {
          fsm = 1;
          ans = true;
          break;
        }
        ans = nao_robot.RobotMoveAround(
            nao_robot.ball_pos.x(), nao_robot.ball_pos.y(), 0.07,
            nao_robot.BallGetAngleToEnemyGate(), 2, 0.03);
        break;
      default:
        fsm = 0;
        break;
      }
      LibXR::Thread::Sleep(20);
    }
  } else if (robot_type == GoalKeeper) {
    ans = 1;
    while (true) {
      float y = nao_robot.ball_pos.y();
      y = clamp(y, 0.35, 0.65);
      ans = nao_robot.RobotGoto(nao_robot.GetOwnGoalKeeperInitX(), y, 0.15);
      if (!ans) {
        ans = true;
        nao_robot.RobotFaceTo(nao_robot.ball_pos.x(), nao_robot.ball_pos.y());
      }
      continue;

      LibXR::Thread::Sleep(20);
    }
  }

  return 0;
}
