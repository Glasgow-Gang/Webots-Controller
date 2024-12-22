#include "kinematic.hpp"
#include "libxr.hpp"
#include "main.hpp"

class RightArm {
public:
  RightArm() {
    /* Objects */
    RShoulder = new LibXR::Kinematic::Object<double>(i_RShoulder);
    RBicep = new LibXR::Kinematic::Object<double>(i_RBicep);
    RElbow = new LibXR::Kinematic::Object<double>(i_RElbow);
    RForeArm = new LibXR::Kinematic::Object<double>(i_RForeArm);
    RWrist = new LibXR::Kinematic::EndPoint<double>(i_RWrist);

    /* Joints */
    LShoulderPitch = new LibXR::Kinematic::Joint<double>(
        a_RShoulderPitch, NaoRobot::nao_robot->Torso, t_Torso_RShoulderPitch,
        RShoulder, t_RShoulderPitch_RShoulder);
    LShoulderRoll = new LibXR::Kinematic::Joint<double>(
        a_RShoulderRoll, RShoulder, t_RShoulder_RShoulderRoll, RBicep,
        t_RShoulderRoll_RBicep);
    LElbowYaw = new LibXR::Kinematic::Joint<double>(
        a_RElbowYaw, RBicep, t_RBicep_RElbowYaw, RElbow, t_RElbowYaw_RElbow);
    LElbowRoll = new LibXR::Kinematic::Joint<double>(
        a_RElbowRoll, RElbow, t_RElbow_RElbowRoll, RForeArm,
        t_RElbowRoll_RForeArm);
    RWristYaw = new LibXR::Kinematic::Joint<double>(a_RWristYaw, RForeArm,
                                                    t_RForeArm_RWristYaw,
                                                    RWrist, t_RWristYaw_RWrist);

    RWrist->SetErrorWeight(
        Eigen::Matrix<double, 6, 1>(1.0, 1.0, 1.0, 0.1, 0.1, 0.1));
    RWrist->SetMaxAngularVelocity(20);
    RWrist->SetMaxLineVelocity(20);

    LShoulderPitch->SetBackwardMult(1.0);
    LShoulderRoll->SetBackwardMult(1.0);
    LElbowYaw->SetBackwardMult(1.0);
    LElbowRoll->SetBackwardMult(1.0);
    RWristYaw->SetBackwardMult(1.0);

    void (*thread_func)(RightArm *) = [](RightArm *l) {
      LibXR::EulerAngle eulr(0., 0., 0.);
      uint32_t count = 0;
      l->RWrist->SetTargetPosition(LibXR::Position(0.25, -0.1, 0.05));
      while (true) {
        count++;
        if (count > 200) {
          count = 0;
        }

        l->RWrist->SetTargetPosition(LibXR::Position(
            0.25,
            -0.15 +
                0.05 * std::sin(static_cast<double>(count) / 100.0 * M_PI * 2),
            0.05 * std::cos(static_cast<double>(count) / 100.0 * M_PI * 2)));

        l->RWrist->SetTargetQuaternion(eulr.toQuaternion());
        l->UpdateFeedback();

        NaoRobot::WaitForwardFinish();
        l->RWrist->CalcBackward(0.001, 100, 0.01, 0.01);

        if (NaoRobot::control_enable) {
          l->Control();
        }

        LibXR::Thread::Sleep(1);
      }
    };

    thread.Create<RightArm *>(this, thread_func, "RightArm", 1024,
                              LibXR::Thread::Priority::MEDIUM);
  }

  void UpdateFeedback() {
    LShoulderPitch->SetState(NaoRobot::nao_robot->JointGetPosition(
        NaoRobot::JointID::RShoulderPitch));
    LShoulderRoll->SetState(NaoRobot::nao_robot->JointGetPosition(
        NaoRobot::JointID::RShoulderRoll));
    LElbowYaw->SetState(
        NaoRobot::nao_robot->JointGetPosition(NaoRobot::JointID::RElbowYaw));
    LElbowRoll->SetState(
        NaoRobot::nao_robot->JointGetPosition(NaoRobot::JointID::RElbowRoll));
    RWristYaw->SetState(
        NaoRobot::nao_robot->JointGetPosition(NaoRobot::JointID::RWristYaw));
    NaoRobot::FeedbackFinish();
  }

  void Control() {
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::RShoulderPitch,
        LShoulderPitch->runtime_.target_angle.angle());
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::RShoulderRoll,
        LShoulderRoll->runtime_.target_angle.angle());
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::RElbowYaw, LElbowYaw->runtime_.target_angle.angle());
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::RElbowRoll,
        LElbowRoll->runtime_.target_angle.angle());
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::RWristYaw, RWristYaw->runtime_.target_angle.angle());
  }

  /* Inertia */
  LibXR::Inertia<double> i_RShoulder =
      LibXR::Inertia(0.07504, 3.10677e-05, 1.39498e-05, 3.30001e-05,
                     -1.2692e-06, -2.99484e-07, 6.04576e-09);
  LibXR::Inertia<double> i_RBicep =
      LibXR::Inertia(0.15777, 0.000103401, 0.00027077, 0.000254529, 5.48849e-05,
                     4.71439e-07, -1.32643e-05);
  LibXR::Inertia<double> i_RElbow =
      LibXR::Inertia(0.06483, 5.59588e-06, 2.66179e-05, 2.76294e-05, 4.21e-09,
                     -1.84e-09, 2.92241e-07);
  LibXR::Inertia<double> i_RForeArm =
      LibXR::Inertia(0.07761, 2.47331e-05, 3.84719e-05, 3.59319e-05,
                     -3.2418e-06, -1.38804e-07, 9.06451e-07);
  LibXR::Inertia<double> i_RWrist =
      LibXR::Inertia(0.18533, 6.86477e-05, 0.000135756, 0.000133228,
                     1.13165e-05, 3.68003e-06, -2.87254e-06);

  /* Transform */
  LibXR::Transform<double> t_Torso_RShoulderPitch = LibXR::Transform<double>(
      LibXR::Quaternion(1.0, 0.0, 0.0, 0.0), LibXR::Position(0.0, -0.098, 0.1));
  LibXR::Transform<double> t_RShoulderPitch_RShoulder =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(-0.00165, 0.02663, 0.00014));
  LibXR::Transform<double> t_RShoulder_RShoulderRoll = LibXR::Transform<double>(
      LibXR::Quaternion(1.0, 0.0, 0.0, 0.0), LibXR::Position(0.0, 0.0, 0.0));
  LibXR::Transform<double> t_RShoulderRoll_RBicep =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(0.02455, -0.00563, 0.0033));
  LibXR::Transform<double> t_RBicep_RElbowYaw =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(0.105, -0.015, 0.0));
  LibXR::Transform<double> t_RElbowYaw_RElbow =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(-0.02744, 0.0, -0.00014));
  LibXR::Transform<double> t_RElbow_RElbowRoll = LibXR::Transform<double>(
      LibXR::Quaternion(1.0, 0.0, 0.0, 0.0), LibXR::Position(0.0, 0.0, 0.0));
  LibXR::Transform<double> t_RElbowRoll_RForeArm =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(0.02556, -0.00281, 0.00076));
  LibXR::Transform<double> t_RForeArm_RWristYaw =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(0.05595, 0.0, 0.0));
  LibXR::Transform<double> t_RWristYaw_RWrist =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(0.03434, 0.00088, 0.00308));

  /* Axis */
  LibXR::Axis<double> a_RShoulderPitch = LibXR::Axis<double>::Y();
  LibXR::Axis<double> a_RShoulderRoll = LibXR::Axis<double>::Z();
  LibXR::Axis<double> a_RElbowYaw = LibXR::Axis<double>::X();
  LibXR::Axis<double> a_RElbowRoll = LibXR::Axis<double>::Z();
  LibXR::Axis<double> a_RWristYaw = LibXR::Axis<double>::X();

  /* Chain */
  LibXR::Kinematic::Joint<double> *LShoulderPitch = nullptr;
  LibXR::Kinematic::Object<double> *RShoulder = nullptr;
  LibXR::Kinematic::Joint<double> *LShoulderRoll = nullptr;
  LibXR::Kinematic::Object<double> *RBicep = nullptr;
  LibXR::Kinematic::Joint<double> *LElbowYaw = nullptr;
  LibXR::Kinematic::Object<double> *RElbow = nullptr;
  LibXR::Kinematic::Joint<double> *LElbowRoll = nullptr;
  LibXR::Kinematic::Object<double> *RForeArm = nullptr;
  LibXR::Kinematic::Joint<double> *RWristYaw = nullptr;
  LibXR::Kinematic::EndPoint<double> *RWrist = nullptr;

  /* Thread... */
  LibXR::Thread thread;

  void SetTargetQuaternion(LibXR::Quaternion<double> target) {
    RWrist->SetTargetQuaternion(target);
  }

  void SetTargetPosition(LibXR::Position<double> target) {
    RWrist->SetTargetPosition(target);
  }

  LibXR::Position<double> GetPosition() {
    return RWrist->runtime_.state.translation;
  }

  LibXR::Quaternion<double> GetQuaternion() {
    return RWrist->runtime_.state.rotation;
  }
};
