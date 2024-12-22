#include "kinematic.hpp"
#include "libxr.hpp"
#include "main.hpp"
#include "transform.hpp"

class RightLeg {
public:
  RightLeg() {
    /* Objects */
    RPelvis = new LibXR::Kinematic::StartPoint<double>(i_RPelvis);
    RHip = new LibXR::Kinematic::Object<double>(i_RHip);
    RThigh = new LibXR::Kinematic::Object<double>(i_RThigh);
    RTibia = new LibXR::Kinematic::Object<double>(i_RTibia);
    RAnkle = new LibXR::Kinematic::Object<double>(i_RAnkle);
    RAnkle_1 = new LibXR::Kinematic::EndPoint<double>(i_RAnkle_1);

    /* Joints */
    RHipYawPitch = new LibXR::Kinematic::Joint<double>(
        a_RHipYawPitch, NaoRobot::nao_robot->Torso, t_Torso_RHipYawPitch,
        RPelvis, t_RHipYawPitch_RPelvis);
    RHipRoll = new LibXR::Kinematic::Joint<double>(
        a_RHipRoll, RPelvis, t_RPelvis_RHipRoll, RHip, t_RHipRoll_RHip);
    RHipPitch = new LibXR::Kinematic::Joint<double>(
        a_RHipPitch, RHip, t_RHip_RHipPitch, RThigh, t_RHipPitch_RThigh);
    RKneePitch = new LibXR::Kinematic::Joint<double>(
        a_RKneePitch, RThigh, t_RThigh_RKneePitch, RTibia, t_RKneePitch_RTibia);
    RAnklePitch = new LibXR::Kinematic::Joint<double>(
        a_RAnklePitch, RTibia, t_RTibia_RAnklePitch, RAnkle,
        t_RAnklePitch_RAnkle);
    RAnkleRoll = new LibXR::Kinematic::Joint<double>(
        a_RAnkleRoll, RAnkle, t_RAnkle_RAnkleRoll, RAnkle_1,
        t_RAnkleRoll_RAnkle_1);

    RAnkle_1->SetErrorWeight(
        Eigen::Matrix<double, 6, 1>(1.0, 1.0, 1.0, 1.0, 1.0, 1.0));
    RAnkle_1->SetMaxAngularVelocity(20);
    RAnkle_1->SetMaxLineVelocity(50);

    RHipYawPitch->SetBackwardMult(1.0);
    RHipRoll->SetBackwardMult(1.0);
    RHipPitch->SetBackwardMult(1.0);
    RKneePitch->SetBackwardMult(1.0);
    RAnklePitch->SetBackwardMult(1.0);
    RAnkleRoll->SetBackwardMult(1.0);

    void (*thread_func)(RightLeg *) = [](RightLeg *l) {
      l->RAnkle_1->SetTargetPosition(
          LibXR::Position(-0.003, -0.0540065, -0.356075));
      l->RAnkle_1->SetTargetQuaternion(LibXR::Quaternion(1., 0., 0., 0.));

      while (true) {
        l->UpdateFeedback();

        NaoRobot::WaitForwardFinish();
        l->RAnkle_1->CalcBackward(1.0, 100, 0.01, 0.2);

        if (NaoRobot::control_enable) {
          l->Control();
        }

        LibXR::Thread::Sleep(1);
      }
    };

    thread.Create<RightLeg *>(this, thread_func, "RightLeg", 1024,
                              LibXR::Thread::Priority::MEDIUM);
  }

  void UpdateFeedback() {
    RHipYawPitch->SetState(
        NaoRobot::nao_robot->JointGetPosition(NaoRobot::JointID::RHipYawPitch));
    RHipRoll->SetState(
        NaoRobot::nao_robot->JointGetPosition(NaoRobot::JointID::RHipRoll));
    RHipPitch->SetState(
        NaoRobot::nao_robot->JointGetPosition(NaoRobot::JointID::RHipPitch));
    RKneePitch->SetState(
        NaoRobot::nao_robot->JointGetPosition(NaoRobot::JointID::RKneePitch));
    RAnklePitch->SetState(
        NaoRobot::nao_robot->JointGetPosition(NaoRobot::JointID::RAnklePitch));
    RAnkleRoll->SetState(
        NaoRobot::nao_robot->JointGetPosition(NaoRobot::JointID::RAnkleRoll));
    NaoRobot::FeedbackFinish();
  }

  void Control() {
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::RHipYawPitch,
        RHipYawPitch->runtime_.target_angle.angle());
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::RHipRoll, RHipRoll->runtime_.target_angle.angle());
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::RHipPitch, RHipPitch->runtime_.target_angle.angle());
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::RKneePitch,
        RKneePitch->runtime_.target_angle.angle());
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::RAnklePitch,
        RAnklePitch->runtime_.target_angle.angle());
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::RAnkleRoll,
        RAnkleRoll->runtime_.target_angle.angle());
  }

  /* Inertia */
  LibXR::Inertia<double> i_RPelvis =
      LibXR::Inertia(0.06981, 2.3407e-05, 4.76355e-05, 4.97021e-05, 1.07922e-06,
                     2.76058e-06, -1.76003e-06);
  LibXR::Inertia<double> i_RHip =
      LibXR::Inertia(0.13053, 2.41106e-05, 6.34892e-05, 5.67694e-05,
                     -6.08684e-07, -1.99137e-07, 6.33119e-06);
  LibXR::Inertia<double> i_RThigh =
      LibXR::Inertia(0.38968, 0.000509844, 0.000465358, 0.000301098,
                     2.11296e-06, -7.91029e-06, 5.64129e-05);
  LibXR::Inertia<double> i_RTibia =
      LibXR::Inertia(0.29142, 0.000470586, 0.000412654, 0.000185772,
                     3.60392e-06, 7.13013e-06, -2.86648e-05);
  LibXR::Inertia<double> i_RAnkle =
      LibXR::Inertia(0.13416, 3.22034e-05, 6.7943e-05, 5.48269e-0, -8.83212e-09,
                     0.0, 4.27549e-06);
  LibXR::Inertia<double> i_RAnkle_1 =
      LibXR::Inertia(0.16184, 9.78911e-05, 0.000369977, 0.000419417,
                     7.88039e-06, 1.44233e-06, 6.1279e-06);

  /* Transform */
  LibXR::Transform<double> t_Torso_RHipYawPitch =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(0., -0.05, -0.085));
  LibXR::Transform<double> t_RHipYawPitch_RPelvis =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(-0.00781, 0.01114, 0.02661));
  LibXR::Transform<double> t_RPelvis_RHipRoll = LibXR::Transform<double>(
      LibXR::Quaternion(1.0, 0.0, 0.0, 0.0), LibXR::Position(0.0, 0.0, 0.0));
  LibXR::Transform<double> t_RHipRoll_RHip =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(-0.01549, -0.00029, -0.00515));
  LibXR::Transform<double> t_RHip_RHipPitch = LibXR::Transform<double>(
      LibXR::Quaternion(1.0, 0.0, 0.0, 0.0), LibXR::Position(0.0, 0.0, 0.0));
  LibXR::Transform<double> t_RHipPitch_RThigh =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(0.00138, -0.00221, -0.05373));
  LibXR::Transform<double> t_RThigh_RKneePitch = LibXR::Transform<double>(
      LibXR::Quaternion(1.0, 0.0, 0.0, 0.0), LibXR::Position(0.0, 0.0, -0.1));
  LibXR::Transform<double> t_RKneePitch_RTibia =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(0.00453, -0.00225, -0.04936));
  LibXR::Transform<double> t_RTibia_RAnklePitch =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(0.0, 0.0, -0.1029));
  LibXR::Transform<double> t_RAnklePitch_RAnkle =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(0.00045, -0.00029, 0.00685));
  LibXR::Transform<double> t_RAnkle_RAnkleRoll = LibXR::Transform<double>(
      LibXR::Quaternion(1.0, 0.0, 0.0, 0.0), LibXR::Position(0.0, 0.0, 0.0));
  LibXR::Transform<double> t_RAnkleRoll_RAnkle_1 =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(0.02542, -0.0033, -0.03239));

  /* Axis */
  LibXR::Axis<double> a_RHipYawPitch =
      LibXR::Axis<double>(0.0, 0.707106, 0.707106);
  LibXR::Axis<double> a_RHipRoll = LibXR::Axis<double>::X();
  LibXR::Axis<double> a_RHipPitch = LibXR::Axis<double>::Y();
  LibXR::Axis<double> a_RKneePitch = LibXR::Axis<double>::Y();
  LibXR::Axis<double> a_RAnklePitch = LibXR::Axis<double>::Y();
  LibXR::Axis<double> a_RAnkleRoll = LibXR::Axis<double>::X();

  /* Chain */
  LibXR::Kinematic::Joint<double> *RHipYawPitch = nullptr;
  LibXR::Kinematic::Object<double> *RPelvis = nullptr;
  LibXR::Kinematic::Joint<double> *RHipRoll = nullptr;
  LibXR::Kinematic::Object<double> *RHip = nullptr;
  LibXR::Kinematic::Joint<double> *RHipPitch = nullptr;
  LibXR::Kinematic::Object<double> *RThigh = nullptr;
  LibXR::Kinematic::Joint<double> *RKneePitch = nullptr;
  LibXR::Kinematic::Object<double> *RTibia = nullptr;
  LibXR::Kinematic::Joint<double> *RAnklePitch = nullptr;
  LibXR::Kinematic::Object<double> *RAnkle = nullptr;
  LibXR::Kinematic::Joint<double> *RAnkleRoll = nullptr;
  LibXR::Kinematic::EndPoint<double> *RAnkle_1 = nullptr;

  /* Thread... */
  LibXR::Thread thread;

  void SetTargetQuaternion(LibXR::Quaternion<double> target) {
    RAnkle_1->SetTargetQuaternion(target);
  }

  void SetTargetPosition(LibXR::Position<double> target) {
    RAnkle_1->SetTargetPosition(target);
  }

  LibXR::Position<double> GetPosition() {
    return RAnkle_1->runtime_.state.translation;
  }

  LibXR::Quaternion<double> GetQuaternion() {
    return RAnkle_1->runtime_.state.rotation;
  }
};
