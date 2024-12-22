#pragma once

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

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
#include "params.hpp"

#include "kinematic.hpp"
#include "thread.hpp"

#define PHALANX_MAX 8

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
  enum class DistanceSensorID { LEFT, RIGHT, NUMBER };

  enum class JointID {
    LShoulderPitch,
    LShoulderRoll,
    LElbowYaw,
    LElbowRoll,
    LHipYawPitch,
    LHipRoll,
    LHipPitch,
    LKneePitch,
    LAnklePitch,
    LAnkleRoll,
    LWristYaw,
    RHipYawPitch,
    RHipRoll,
    RHipPitch,
    RKneePitch,
    RAnklePitch,
    RAnkleRoll,
    RShoulderPitch,
    RShoulderRoll,
    RElbowYaw,
    RElbowRoll,
    RWristYaw,
    HeadYaw,
    HeadPitch,
    NUMBER
  };

  NaoRobot() : robot_(_libxr_webots_robot_handle) {
    nao_robot = this;
    timeStep = robot_->getBasicTimeStep();

    /* Initialize devices */
    CameraTop = robot_->getCamera("CameraTop");
    CameraBottom = robot_->getCamera("CameraBottom");
    CameraTop->enable(4 * timeStep);
    CameraBottom->enable(4 * timeStep);

    /* Accelerometer */
    accelerometer = robot_->getAccelerometer("accelerometer");
    accelerometer->enable(timeStep);

    /* Gyro */
    gyro = robot_->getGyro("gyro");
    gyro->enable(timeStep);

    /* Inertial Unit */
    inertialUnit = robot_->getInertialUnit("inertial unit");
    inertialUnit->enable(timeStep);

    /* Ultrasound sensors */
    us[0] = robot_->getDistanceSensor("Sonar/Left");
    us[1] = robot_->getDistanceSensor("Sonar/Right");
    for (auto sensor : us) {
      sensor->enable(timeStep);
    }

    /* Touch sensors and LEDs */
    leds.push_back(robot_->getLED("ChestBoard/Led"));
    leds.push_back(robot_->getLED("RFoot/Led"));
    leds.push_back(robot_->getLED("LFoot/Led"));
    leds.push_back(robot_->getLED("Face/Led/Right"));
    leds.push_back(robot_->getLED("Face/Led/Left"));
    leds.push_back(robot_->getLED("Ears/Led/Right"));
    leds.push_back(robot_->getLED("Ears/Led/Left"));

    /* Motors */
    RShoulderPitch = robot_->getMotor("RShoulderPitch");
    LShoulderPitch = robot_->getMotor("LShoulderPitch");

    for (int i = 0; i < PHALANX_MAX; i++) {
      lPhalanx[i] = robot_->getMotor("LPhalanx" + std::to_string(i + 1));
      rPhalanx[i] = robot_->getMotor("RPhalanx" + std::to_string(i + 1));
    }

    for (int i = 0; i < static_cast<int>(NaoRobot::JointID::NUMBER); i++) {
      auto actuator_name =
          magic_enum::enum_name(static_cast<NaoRobot::JointID>(i));
      joint[i] = robot_->getMotor(std::string(actuator_name).c_str());
      joint_sensors[i] = robot_->getPositionSensor(
          std::string(actuator_name).c_str() + std::string("S"));
      joint_sensors[i]->enable(timeStep);
      joint[i]->setControlPID(15.0, 2.0, 0.);
    }

    keyboard.enable(timeStep);

    LibXR::Inertia<double> i_Torso =
        LibXR::Inertia(1.04956, 0.00308361, 0.0028835, 0.0015924, 1.43116e-05,
                       -2.70793e-05, -3.30211e-05);

    Torso = new LibXR::Kinematic::StartPoint<double>(i_Torso);

    void (*thread_fun)(LibXR::Kinematic::StartPoint<double> *) =
        [](LibXR::Kinematic::StartPoint<double> *start) {
          while (true) {
            nao_robot->WaitFeedback();
            start->CalcForward();
            start->CalcCenterOfMass();
            nao_robot->ForwardFinish();
            LibXR::Thread::Sleep(1);
          }
        };

    thread.Create(Torso, thread_fun, "main", 4096,
                  LibXR::Thread::Priority::REALTIME);
  }

  void JointPositionControl(JointID id, double position) {
    joint[static_cast<int>(id)]->setPosition(position);
  }

  void JointVelocityControl(JointID id, double velocity) {
    joint[static_cast<int>(id)]->setVelocity(velocity);
  }

  void JointTorqueControl(JointID id, double torque) {
    joint[static_cast<int>(id)]->setForce(torque);
  }

  double JointGetPosition(JointID id) {
    return joint_sensors[static_cast<int>(id)]->getValue();
  }

  double GetDistanceSensorValue(DistanceSensorID id) {
    return us[static_cast<int>(id)]->getValue();
  }

  void UpdateEulr() {
    spdlog::info("# Get Body Euler angles");

    auto eulr = inertialUnit->getRollPitchYaw();

    body_posture_.roll = eulr[0];
    body_posture_.pitch = eulr[1];
    body_posture_.yaw = eulr[2];

    spdlog::debug("Pitch: {}", body_posture_.pitch);
    spdlog::debug("Yaw: {}", body_posture_.yaw);
    spdlog::debug("Roll: {}", body_posture_.roll);
  }

  void HeadControl();

  static void WaitFeedback() {
    for (int i = 0; i < 4; i++) {
      nao_robot->wait_feedback.Wait(10);
    }
  }

  static void WaitForwardFinish() { nao_robot->forward_finish.Wait(2); }

  static void FeedbackFinish() { nao_robot->wait_feedback.Post(); }

  static void ForwardFinish() { nao_robot->forward_finish.Broadcast(); }

  webots::Robot *robot_;

  webots::Camera *CameraTop, *CameraBottom;
  webots::Accelerometer *accelerometer;
  webots::Gyro *gyro;
  webots::InertialUnit *inertialUnit;
  webots::DistanceSensor
      *us[static_cast<int>(NaoRobot::DistanceSensorID::NUMBER)];
  webots::Motor *rPhalanx[PHALANX_MAX], *lPhalanx[PHALANX_MAX];
  webots::Motor *RShoulderPitch, *LShoulderPitch;
  webots::Motor *joint[static_cast<int>(NaoRobot::JointID::NUMBER)];
  webots::PositionSensor
      *joint_sensors[static_cast<int>(NaoRobot::JointID::NUMBER)];
  std::vector<webots::LED *> leds;
  webots::Keyboard keyboard;

  double target_position_[static_cast<int>(NaoRobot::JointID::NUMBER)] = {0};

  struct {
    double pitch, yaw, roll;
  } body_posture_;

  int timeStep;

  static NaoRobot *nao_robot;

  LibXR::Kinematic::StartPoint<double> *Torso = nullptr;

  LibXR::Semaphore wait_feedback;

  LibXR::ConditionVar forward_finish;

  LibXR::Thread thread;

  static bool control_enable;
};
