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
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/utils/Motion.hpp>

#include "magic_enum.hpp"

#define PHALANX_MAX 8

class NaoRobot : public webots::Robot {
 public:
  enum class DistanceSensorID { LEFT, RIGHT, NUMBER };

  enum class ActuatorID {
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

    for (int i = 0; i < static_cast<int>(NaoRobot::ActuatorID::NUMBER); i++) {
      auto actuator_name =
          magic_enum::enum_name(static_cast<NaoRobot::ActuatorID>(i));
      body[i] = getMotor(std::string(actuator_name).c_str());
    }

    getMotor("LShoulderPitch");

    keyboard.enable(timeStep);
  }

  void BodyPositionControl(ActuatorID id, double position) {
    body[static_cast<int>(id)]->setPosition(position);
  }

  void BodyVelocityControl(ActuatorID id, double velocity) {
    body[static_cast<int>(id)]->setVelocity(velocity);
  }

  void BodyTorqueControl(ActuatorID id, double torque) {
    body[static_cast<int>(id)]->setForce(torque);
  }

  double GetDistanceSensorValue(DistanceSensorID id) {
    return us[static_cast<int>(id)]->getValue();
  }

  webots::Camera *CameraTop, *CameraBottom;
  webots::Accelerometer *accelerometer;
  webots::Gyro *gyro;
  webots::InertialUnit *inertialUnit;
  webots::DistanceSensor
      *us[static_cast<int>(NaoRobot::DistanceSensorID::NUMBER)];
  webots::Motor *rPhalanx[PHALANX_MAX], *lPhalanx[PHALANX_MAX];
  webots::Motor *RShoulderPitch, *LShoulderPitch;
  webots::Motor *body[static_cast<int>(NaoRobot::ActuatorID::NUMBER)];
  std::vector<webots::LED *> leds;
  webots::Keyboard keyboard;

  int timeStep;
};
