#pragma once

#include "condition_var.hpp"
#include "libxr.hpp"

#include <iostream>
#include <string>
#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/utils/Motion.hpp>

#include "libxr_def.hpp"
#include "libxr_system.hpp"
#include "magic_enum.hpp"

#include "kinematic.hpp"
#include "thread.hpp"

class NaoRobot;

enum class Motion {
  Backwards,
  SideStepLeft,
  TurnLeft40,
  Forwards50,
  SideStepRight,
  TurnLeft60,
  Forwards,
  StandUpFromFront,
  TurnRight40,
  HandWave,
  TaiChi,
  TurnRight60,
  Shoot,
  TurnLeft180,
  WipeForehead,
};

class NaoRobot {
public:
  NaoRobot() : robot_(_libxr_webots_robot_handle) {
    nao_robot = this;
    timeStep = robot_->getBasicTimeStep();
  }
  int timeStep;

  webots::Robot *robot_;

  static NaoRobot *nao_robot;

  LibXR::Thread thread;
};
