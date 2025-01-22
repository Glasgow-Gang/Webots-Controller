#pragma once

#include <cstdio>
#include <iostream>
#include <string>
#include <unistd.h>
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/utils/Motion.hpp>

#include "async.hpp"
#include "circle_value.hpp"
#include "libxr_def.hpp"
#include "magic_enum.hpp"
#include "transform.hpp"

#include "thread.hpp"
#include "webots/Node.hpp"

#define GATE_1_X (0.04)
#define GATE_2_X (0.94)

class NaoRobot;

enum MotionFile {
  Backwards,
  ForwardLoop,
  Forwards50,
  Forwards,
  ForwardsSprint,
  HandWave,
  LongPass,
  RightShoot,
  Shoot,
  SidePass_Left,
  SidePass_Right,
  SideStepLeftLoop,
  SideStepLeft,
  SideStepRightLoop,
  SideStepRight,
  StandInit,
  Stand,
  StandUpFromBack,
  StandUpFromFront,
  TurnLeft10,
  TurnLeft10_V2,
  TurnLeft180,
  TurnLeft20,
  TurnLeft30,
  TurnLeft40,
  TurnLeft60,
  TurnRight10,
  TurnRight10_V2,
  TurnRight20,
  TurnRight40,
  TurnRight60,
  MOTION_FILE_COUNT
};

class NaoRobot {
public:
  NaoRobot(double gate_addr = GATE_1_X)
      : gate_addr(gate_addr), async(4096, LibXR::Thread::Priority::REALTIME) {
    nao_robot = this;
    timeStep = supervisor.getBasicTimeStep();
    robot_node = supervisor.getFromDef("PLAYER_1_0");
    ball_node = supervisor.getFromDef("BALL");
    robot_translation_field = robot_node->getField("translation");
    robot_rotation_field = robot_node->getField("rotation");
    ball_translation_field = ball_node->getField("translation");

    constexpr auto enum_names = magic_enum::enum_names<MotionFile>();

    for (int i = 0; i < MOTION_FILE_COUNT; i++) {
      motion[i] = new webots::Motion(std::string("../motions/") +
                                     std::string(enum_names[i]) + ".motion");
    }

    async_callback = LibXR::Callback<LibXR::ASync *>::Create(AsyncMotionPlayFun,
                                                             &current_motion);

    ASSERT(async.GetStatus() == LibXR::ASync::Status::REDAY);

    void (*thread_fun)(NaoRobot *) = [](NaoRobot *robot) {
      while (true) {
        robot->Update();
        LibXR::Thread::Sleep(robot->timeStep);
      }
    };

    thread.Create(this, thread_fun, "sim2d", 4096,
                  LibXR::Thread::Priority::REALTIME);
  }

  void Update() {
    const double *position = robot_translation_field->getSFVec3f();
    robot_pos = LibXR::Position<double>((position[0] + 5.0) / 10.0,
                                        1.0 - (position[1] + 3.5) / 7.0, 0);

    const double *rotation = robot_rotation_field->getSFRotation();
    robot_angle =
        LibXR::RotationMatrix<double>(
            Eigen::AngleAxis<double>(
                -rotation[3],
                Eigen::Vector3d(rotation[0], rotation[1], rotation[2]))
                .toRotationMatrix())
            .toEulerAngleZYX();

    const double *ball_position = ball_translation_field->getSFVec3f();
    ball_pos = LibXR::Position<double>((ball_position[0] + 5.0) / 10.0,
                                       1.0 - (ball_position[1] + 3.5) / 7.0, 0);
  }

  void PlayMotion(MotionFile motion_file) {
    auto status = async.GetStatus();
    if (status == LibXR::ASync::Status::DONE ||
        status == LibXR::ASync::Status::REDAY) {
      current_motion = motion_file;
      async.AssignJob(async_callback);
    }
  }

  void RobotTurn(float target_angle) {
    float angle = LibXR::CycleValue<double>(nao_robot->robot_angle.yaw_) -
                  LibXR::CycleValue<double>(target_angle);
    angle = -angle * 180 / M_PI;

    if (angle > 0) {
      if (angle > 60) {
        PlayMotion(MotionFile::TurnRight60);
      } else if (angle > 40) {
        PlayMotion(MotionFile::TurnRight40);
      } else if (angle > 20) {
        PlayMotion(MotionFile::TurnRight20);
      } else if (angle > 10) {
        PlayMotion(MotionFile::TurnRight20);
      }
    } else {
      if (angle < -175) {
        PlayMotion(MotionFile::TurnLeft180);
      } else if (angle < -60) {
        PlayMotion(MotionFile::TurnLeft60);
      } else if (angle < -40) {
        PlayMotion(MotionFile::TurnLeft40);
      } else if (angle < -30) {
        PlayMotion(MotionFile::TurnLeft30);
      } else if (angle < -20) {
        PlayMotion(MotionFile::TurnLeft20);
      } else if (angle < -10) {
        PlayMotion(MotionFile::TurnLeft20);
      }
    }
  }

  void RobotFaceTo(double x, double y) {
    double target_angle = LibXR::CycleValue<double>(
        atan2(y - nao_robot->robot_pos.y(), x - nao_robot->robot_pos.x()));
    this->target_angle = target_angle;
    RobotTurn(target_angle);
  }

  double RobotGetDistanceTo(double x, double y) {
    return sqrt(
        (x - nao_robot->robot_pos.x()) * (x - nao_robot->robot_pos.x()) +
        (y - nao_robot->robot_pos.y()) * (y - nao_robot->robot_pos.y()));
  }

  double RobotGetAngleTo(double x, double y) {
    return LibXR::CycleValue<double>(
        atan2(y - nao_robot->robot_pos.y(), x - nao_robot->robot_pos.x()));
  }

  void RobotGoto(double &x, double &y, double min_error = 0.01) {
    auto distance = RobotGetDistanceTo(x, y);

    this->target_x = x;
    this->target_y = y;

    if (distance > min_error) {
      if (std::abs(
              LibXR::CycleValue<double>(atan2(y - nao_robot->robot_pos.y(),
                                              x - nao_robot->robot_pos.x())) -
              nao_robot->robot_angle.yaw_) < M_PI / 18) {
        if (distance > 0.2) {
          PlayMotion(MotionFile::Forwards50);
        } else {
          PlayMotion(MotionFile::Forwards);
        }
      } else {
        RobotFaceTo(x, y);
      }
    } else {
      RobotFaceTo(x, y);
    }
  }

  void RobotMoveAround(double x, double y, double radius, double angle) {
    // 计算极坐标目标点的全局坐标
    double target_x = x + radius * cos(angle);
    double target_y = y + radius * sin(angle);

    if (RobotGetDistanceTo(target_x, target_y) >
        0.01) { // 移动到计算出的目标位置
      RobotGoto(target_x, target_y);
    } else {
      RobotFaceTo(x, y);
    }
  }

  double BallGetAngleToOwnGate() {
    double own_gate_x = gate_addr, own_date_y = 0.5;
    return LibXR::CycleValue<double>(
        atan2(own_date_y - nao_robot->ball_pos.y(),
              own_gate_x - nao_robot->ball_pos.x()));
  }

  double BallGetDistanceToOwnGate() {
    double own_gate_x = gate_addr, own_date_y = 0.5;
    return sqrt((own_gate_x - nao_robot->ball_pos.x()) *
                    (own_gate_x - nao_robot->ball_pos.x()) +
                (own_date_y - nao_robot->ball_pos.y()) *
                    (own_date_y - nao_robot->ball_pos.y()));
  }

  double BallGetAngleToEnemyGate() {
    double enemy_gate_x = GetEnemyGate(), enemy_date_y = 0.5;
    return LibXR::CycleValue<double>(
        M_PI + atan2(enemy_date_y - nao_robot->ball_pos.y(),
                     enemy_gate_x - nao_robot->ball_pos.x()));
  }

  double BallGetDistanceToEnemyGate() {
    double enemy_gate_x = GetEnemyGate(), enemy_date_y = 0.5;
    return sqrt((enemy_gate_x - nao_robot->ball_pos.x()) *
                    (enemy_gate_x - nao_robot->ball_pos.x()) +
                (enemy_date_y - nao_robot->ball_pos.y()) *
                    (enemy_date_y - nao_robot->ball_pos.y()));
  }

  double GetEnemyGate() {
    if (gate_addr == GATE_1_X) {
      return GATE_2_X;
    } else {
      return GATE_1_X;
    }
  }

  static void AsyncMotionPlayFun(bool in_isr, MotionFile *motion_file,
                                 LibXR::ASync *async) {
    UNUSED(in_isr);
    UNUSED(async);

    printf("PlayMotion(%s)\n", magic_enum::enum_name(*motion_file).data());
    nao_robot->motion[*motion_file]->play();
    while (!nao_robot->motion[*motion_file]->isOver()) {
      LibXR::Thread::Sleep(nao_robot->timeStep);
    }
  }

  double gate_addr;

  int timeStep;

  webots::Supervisor supervisor;

  webots::Node *robot_node = nullptr;
  webots::Node *ball_node = nullptr;

  webots::Motion *motion[MOTION_FILE_COUNT];

  static NaoRobot *nao_robot;

  const webots::Field *robot_translation_field;
  const webots::Field *robot_rotation_field;
  const webots::Field *ball_translation_field;

  LibXR::Position<double> robot_pos;
  LibXR::EulerAngle<double> robot_angle;

  LibXR::Position<double> ball_pos;

  LibXR::Thread thread;

  double target_x = 0, target_y = 0;
  double target_angle = 0;

  LibXR::ASync async;
  LibXR::Callback<LibXR::ASync *> async_callback;
  MotionFile current_motion = MotionFile::Forwards;
};
