#include "kinematic.hpp"
#include "libxr.hpp"
#include "main.hpp"

class LeftArm {
public:
  LeftArm() {
    /* Objects */
    Torso = new LibXR::Kinematic::StartPoint<double>(i_Torso);
    LShoulder = new LibXR::Kinematic::Object<double>(i_LShoulder);
    LBicep = new LibXR::Kinematic::Object<double>(i_LBicep);
    LElbow = new LibXR::Kinematic::Object<double>(i_LElbow);
    LForeArm = new LibXR::Kinematic::Object<double>(i_LForeArm);
    LWrist = new LibXR::Kinematic::EndPoint<double>(i_LWrist);

    /* Joints */
    LShoulderPitch = new LibXR::Kinematic::Joint<double>(
        a_LShoulderPitch, Torso, t_Torso_LShoulderPitch, LShoulder,
        t_LShoulderPitch_LShoulder);
    LShoulderRoll = new LibXR::Kinematic::Joint<double>(
        a_LShoulderRoll, LShoulder, t_LShoulder_LShoulderRoll, LBicep,
        t_LShoulderRoll_LBicep);
    LElbowYaw = new LibXR::Kinematic::Joint<double>(
        a_LElbowYaw, LBicep, t_LBicep_LElbowYaw, LElbow, t_LElbowYaw_LElbow);
    LElbowRoll = new LibXR::Kinematic::Joint<double>(
        a_LElbowRoll, LElbow, t_LElbow_LElbowRoll, LForeArm,
        t_LElbowRoll_LForeArm);
    LWristYaw = new LibXR::Kinematic::Joint<double>(a_LWristYaw, LForeArm,
                                                    t_LForeArm_LWristYaw,
                                                    LWrist, t_LWristYaw_LWrist);

    LWrist->SetErrorWeight(
        Eigen::Matrix<double, 6, 1>(1.0, 1.0, 1.0, 0.1, 0.1, 0.1));

    LShoulderPitch->SetBackwardMult(1.0);
    LShoulderRoll->SetBackwardMult(1.0);
    LElbowYaw->SetBackwardMult(1.0);
    LElbowRoll->SetBackwardMult(1.0);
    LWristYaw->SetBackwardMult(1.0);

    void (*thread_func)(LeftArm *) = [](LeftArm *l) {
      LibXR::EulerAngle eulr(0., 0., 0.);
      uint32_t count = 0;
      uint32_t fsm = 0;
      l->LWrist->SetTargetPosition(LibXR::Position(0.25, 0.1, 0.05));
      while (true) {
        count++;
        if (count >= 100) {
          count = 0;
          switch (fsm) {
          case 0:
            l->LWrist->SetTargetPosition(LibXR::Position(0.25, 0.1, 0.1));
            fsm = 1;
            break;
          case 1:
            l->LWrist->SetTargetPosition(LibXR::Position(0.25, 0.1, 0.0));
            fsm = 2;
            break;
          case 2:
            l->LWrist->SetTargetPosition(LibXR::Position(0.25, 0.2, 0.0));

            fsm = 3;
            break;
          case 3:
            fsm = 4;
            l->LWrist->SetTargetPosition(LibXR::Position(0.25, 0.2, 0.1));

            break;
          default:
            fsm = 0;
            break;
          }
        }

        l->LWrist->SetTargetQuaternion(eulr.toQuaternion());
        l->UpdateFeedback();
        l->Torso->CalcForward();
        l->LWrist->CalcBackward(100, 0.01, 0.01);

        l->Control();

        LibXR::Thread::Sleep(1);
      }
    };

    thread.Create<LeftArm *>(this, thread_func, "LeftArm", 1024,
                             LibXR::Thread::Priority::MEDIUM);
  }

  void UpdateFeedback() {
    LShoulderPitch->SetState(NaoRobot::nao_robot->JointGetPosition(
        NaoRobot::JointID::LShoulderPitch));
    LShoulderRoll->SetState(NaoRobot::nao_robot->JointGetPosition(
        NaoRobot::JointID::LShoulderRoll));
    LElbowYaw->SetState(
        NaoRobot::nao_robot->JointGetPosition(NaoRobot::JointID::LElbowYaw));
    LElbowRoll->SetState(
        NaoRobot::nao_robot->JointGetPosition(NaoRobot::JointID::LElbowRoll));
    LWristYaw->SetState(
        NaoRobot::nao_robot->JointGetPosition(NaoRobot::JointID::LWristYaw));
  }

  void Control() {
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::LShoulderPitch,
        LShoulderPitch->runtime_.target_angle.angle());
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::LShoulderRoll,
        LShoulderRoll->runtime_.target_angle.angle());
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::LElbowYaw, LElbowYaw->runtime_.target_angle.angle());
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::LElbowRoll,
        LElbowRoll->runtime_.target_angle.angle());
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::LWristYaw, LWristYaw->runtime_.target_angle.angle());
  }

  /* Inertia */
  LibXR::Inertia<double> i_Torso =
      LibXR::Inertia(1.04956, 0.00308361, 0.0028835, 0.0015924, 1.43116e-05,
                     -2.70793e-05, -3.30211e-05);
  LibXR::Inertia<double> i_LShoulder =
      LibXR::Inertia(0.07504, 3.10677e-05, 1.39498e-05, 3.30001e-05, 1.2692e-06,
                     -2.99484e-07, 6.04576e-09);
  LibXR::Inertia<double> i_LBicep =
      LibXR::Inertia(0.15777, 8.7181e-05, 0.000274712, 0.000241812,
                     -2.53381e-05, 4.71439e-07, -1.4213e-05);
  LibXR::Inertia<double> i_LElbow =
      LibXR::Inertia(0.06483, 5.59588e-06, 2.66179e-05, 2.76294e-05, 4.21e-09,
                     -1.84e-09, 2.92241e-07);
  LibXR::Inertia<double> i_LForeArm =
      LibXR::Inertia(0.07761, 2.46746e-05, 3.83837e-05, 3.59708e-05,
                     3.23152e-06, 3.59708e-05, 1.58221e-06);
  LibXR::Inertia<double> i_LWrist =
      LibXR::Inertia(0.18533, 6.86477e-05, 0.000135756, 0.000133228,
                     1.15465e-07, 2.67539e-06, -2.87254e-06);

  /* Transform */
  LibXR::Transform<double> t_Torso_LShoulderPitch = LibXR::Transform<double>(
      LibXR::Quaternion(1.0, 0.0, 0.0, 0.0), LibXR::Position(0.0, 0.098, 0.1));
  LibXR::Transform<double> t_LShoulderPitch_LShoulder =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(-0.00165, -0.02663, 0.00014));
  LibXR::Transform<double> t_LShoulder_LShoulderRoll = LibXR::Transform<double>(
      LibXR::Quaternion(1.0, 0.0, 0.0, 0.0), LibXR::Position(0.0, 0.0, 0.0));
  LibXR::Transform<double> t_LShoulderRoll_LBicep =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(0.02455, 0.00563, 0.0033));
  LibXR::Transform<double> t_LBicep_LElbowYaw =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(0.105, 0.015, 0.0));
  LibXR::Transform<double> t_LElbowYaw_LElbow =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(-0.02744, 0.0, -0.00014));
  LibXR::Transform<double> t_LElbow_LElbowRoll = LibXR::Transform<double>(
      LibXR::Quaternion(1.0, 0.0, 0.0, 0.0), LibXR::Position(0.0, 0.0, 0.0));
  LibXR::Transform<double> t_LElbowRoll_LForeArm =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(0.02556, 0.00281, 0.00076));
  LibXR::Transform<double> t_LForeArm_LWristYaw =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(0.05595, 0.0, 0.0));
  LibXR::Transform<double> t_LWristYaw_LWrist =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(0.03434, -0.00088, 0.00308));

  /* Axis */
  LibXR::Axis<double> a_LShoulderPitch = LibXR::Axis<double>::Y();
  LibXR::Axis<double> a_LShoulderRoll = LibXR::Axis<double>::Z();
  LibXR::Axis<double> a_LElbowYaw = LibXR::Axis<double>::X();
  LibXR::Axis<double> a_LElbowRoll = LibXR::Axis<double>::Z();
  LibXR::Axis<double> a_LWristYaw = LibXR::Axis<double>::X();

  /* Chain */
  LibXR::Kinematic::StartPoint<double> *Torso = nullptr;
  LibXR::Kinematic::Joint<double> *LShoulderPitch = nullptr;
  LibXR::Kinematic::Object<double> *LShoulder = nullptr;
  LibXR::Kinematic::Joint<double> *LShoulderRoll = nullptr;
  LibXR::Kinematic::Object<double> *LBicep = nullptr;
  LibXR::Kinematic::Joint<double> *LElbowYaw = nullptr;
  LibXR::Kinematic::Object<double> *LElbow = nullptr;
  LibXR::Kinematic::Joint<double> *LElbowRoll = nullptr;
  LibXR::Kinematic::Object<double> *LForeArm = nullptr;
  LibXR::Kinematic::Joint<double> *LWristYaw = nullptr;
  LibXR::Kinematic::EndPoint<double> *LWrist = nullptr;

  /* Thread... */
  LibXR::Thread thread;
};
