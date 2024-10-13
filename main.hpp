#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

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

#include "magic_enum.hpp"
#include "params.hpp"

#define PHALANX_MAX 8

class NaoRobot : public webots::Robot {
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
    HeadYaw,
    HeadPitch,
    NUMBER
  };

  NaoRobot() {
    timeStep = getBasicTimeStep();

    /* Initialize devices */
    CameraTop = getCamera("CameraTop");
    CameraBottom = getCamera("CameraBottom");
    CameraTop->enable(4 * timeStep);
    CameraBottom->enable(4 * timeStep);

    /* Accelerometer */
    accelerometer = getAccelerometer("accelerometer");
    accelerometer->enable(timeStep);

    /* Gyro */
    gyro = getGyro("gyro");
    gyro->enable(timeStep);

    /* Inertial Unit */
    inertialUnit = getInertialUnit("inertial unit");
    inertialUnit->enable(timeStep);

    /* Ultrasound sensors */
    us[0] = getDistanceSensor("Sonar/Left");
    us[1] = getDistanceSensor("Sonar/Right");
    for (auto sensor : us) {
      sensor->enable(timeStep);
    }

    /* Touch sensors and LEDs */
    leds.push_back(getLED("ChestBoard/Led"));
    leds.push_back(getLED("RFoot/Led"));
    leds.push_back(getLED("LFoot/Led"));
    leds.push_back(getLED("Face/Led/Right"));
    leds.push_back(getLED("Face/Led/Left"));
    leds.push_back(getLED("Ears/Led/Right"));
    leds.push_back(getLED("Ears/Led/Left"));

    /* Motors */
    RShoulderPitch = getMotor("RShoulderPitch");
    LShoulderPitch = getMotor("LShoulderPitch");

    for (int i = 0; i < PHALANX_MAX; i++) {
      lPhalanx[i] = getMotor("LPhalanx" + std::to_string(i + 1));
      rPhalanx[i] = getMotor("RPhalanx" + std::to_string(i + 1));
    }

    for (int i = 0; i < static_cast<int>(NaoRobot::JointID::NUMBER); i++) {
      auto actuator_name =
          magic_enum::enum_name(static_cast<NaoRobot::JointID>(i));
      joint[i] = getMotor(std::string(actuator_name).c_str());
      joint_sensors[i] = getPositionSensor(std::string(actuator_name).c_str() +
                                           std::string("S"));
      joint_sensors[i]->enable(timeStep);
    }

    getMotor("LShoulderPitch");

    keyboard.enable(timeStep);
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
};
