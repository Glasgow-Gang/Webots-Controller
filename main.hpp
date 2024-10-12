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
    auto eulr = inertialUnit->getRollPitchYaw();
    float eulr_0 = eulr[0] - M_PI_2;
    if (eulr[0] > M_PI) eulr_0 -= 2 * M_PI;
    if (eulr[0] < -M_PI) eulr_0 += 2 * M_PI;
    body_posture_.pitch = eulr[1];
    body_posture_.yaw = eulr[2];
    body_posture_.roll = eulr_0;

    std::cout << "Pitch: " << body_posture_.pitch << std::endl;
    std::cout << "Yaw: " << body_posture_.yaw << std::endl;
    std::cout << "Roll: " << body_posture_.roll << std::endl;
  }

  void HeadControl() {
    double pitch =
        JointGetPosition(NaoRobot::JointID::HeadPitch) -
        target_position_[static_cast<int>(NaoRobot::JointID::HeadPitch)];
    double yaw = JointGetPosition(NaoRobot::JointID::HeadYaw) -
                 target_position_[static_cast<int>(NaoRobot::JointID::HeadYaw)];

    static Eigen::Matrix<double, 6, 1> old_x =
        (Eigen::Matrix<double, 6, 1>() << pitch, yaw, 0, 0, 0, 0).finished();

    Eigen::Matrix<double, 6, 1> x;
    x << pitch, yaw, 0, (pitch - old_x(0, 0)) / timeStep * 1000,
        (yaw - old_x(1, 0)) / timeStep * 1000, 0;

    old_x = x;

    std::cout << "x = " << x << std::endl;
    std::cout << "K = " << K_head << std::endl;

    double psi_beta = body_posture_.yaw;      // Yaw angle of the body
    double theta_beta = body_posture_.pitch;  // Pitch angle of the body
    double phi_beta = body_posture_.roll;     // Roll angle of the body

    double theta_yaw =
        JointGetPosition(NaoRobot::JointID::HeadYaw);  // Yaw angle of the head
    double theta_pitch = JointGetPosition(
        NaoRobot::JointID::HeadPitch);  // Pitch angle of the head
    double theta_roll = 0;              // Roll angle of the head

    // Calculate the rotation matrix C_beta^alpha for the body
    Eigen::Matrix3d C_beta_alpha;
    C_beta_alpha << cos(theta_beta) * cos(psi_beta),
        cos(theta_beta) * sin(psi_beta), -sin(theta_beta),
        sin(phi_beta) * sin(theta_beta) * cos(psi_beta) -
            cos(phi_beta) * sin(psi_beta),
        sin(phi_beta) * sin(theta_beta) * sin(psi_beta) +
            cos(phi_beta) * cos(psi_beta),
        sin(phi_beta) * cos(theta_beta),
        cos(phi_beta) * sin(theta_beta) * cos(psi_beta) +
            sin(phi_beta) * sin(psi_beta),
        cos(phi_beta) * sin(theta_beta) * sin(psi_beta) -
            sin(phi_beta) * cos(psi_beta),
        cos(phi_beta) * cos(theta_beta);

    // Calculate the rotation matrix C_gamma^beta for the head
    Eigen::Matrix3d C_gamma_beta;
    C_gamma_beta << cos(theta_pitch) * cos(theta_yaw),
        cos(theta_pitch) * sin(theta_yaw), -sin(theta_pitch),
        sin(theta_roll) * sin(theta_pitch) * cos(theta_yaw) -
            cos(theta_roll) * sin(theta_yaw),
        sin(theta_roll) * sin(theta_pitch) * sin(theta_yaw) +
            cos(theta_roll) * cos(theta_yaw),
        sin(theta_roll) * cos(theta_pitch),
        cos(theta_roll) * sin(theta_pitch) * cos(theta_yaw) +
            sin(theta_roll) * sin(theta_yaw),
        cos(theta_roll) * sin(theta_pitch) * sin(theta_yaw) -
            sin(theta_roll) * cos(theta_yaw),
        cos(theta_roll) * cos(theta_pitch);

    // Calculate the overall rotation matrix C_gamma^alpha
    Eigen::Matrix3d C_gamma_alpha = C_beta_alpha * C_gamma_beta;

    // Extract elements from the rotation matrix
    // double r_11 = C_gamma_alpha(0, 0);
    // double r_12 = C_gamma_alpha(0, 1);
    // double r_13 = C_gamma_alpha(0, 2);
    // double r_21 = C_gamma_alpha(1, 0);
    // double r_22 = C_gamma_alpha(1, 1);
    // double r_23 = C_gamma_alpha(1, 2);
    double r_31 = C_gamma_alpha(2, 0);
    double r_32 = C_gamma_alpha(2, 1);
    double r_33 = C_gamma_alpha(2, 2);

    // Calculate the Euler angles
    // double psi_gamma = atan2(r_21, r_11);  // Yaw 角
    double theta_gamma =
        atan2(-r_31, sqrt(r_32 * r_32 + r_33 * r_33));  // Pitch 角
    // double theta_beta_calculated = atan2(r_32, r_33);   // Roll 角

    Eigen::Matrix<double, 3, 1> u_k = K_head * x;
    Eigen::Matrix<double, 3, 1> u_ff =
        (Eigen::Matrix<double, 3, 1>()
             << -I_head_pitch / L_head_pitch * g * sin(theta_gamma),
         0, 0)
            .finished();
    std::cout << "u_k = " << u_k << std::endl;
    std::cout << "u_ff = " << u_ff << std::endl;

    Eigen::Matrix<double, 3, 1> u = u_k + u_ff;
    std::cout << "u = " << u << std::endl;

    JointTorqueControl(NaoRobot::JointID::HeadPitch, -u(0, 0));
    JointTorqueControl(NaoRobot::JointID::HeadYaw, -u(1, 0));
    // JointTorqueControl(NaoRobot::JointID::HeadRoll, u(2, 0));
  }

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

  struct __attribute__((packed)) {
    double pitch, yaw, roll;
  } body_posture_;

  int timeStep;
};
