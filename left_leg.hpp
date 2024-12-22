#include "kinematic.hpp"
#include "libxr.hpp"
#include "main.hpp"
#include "transform.hpp"

class LeftLeg {
public:
  LeftLeg() {
    /* Objects */
    LPelvis = new LibXR::Kinematic::StartPoint<double>(i_LPelvis);
    LHip = new LibXR::Kinematic::Object<double>(i_LHip);
    LThigh = new LibXR::Kinematic::Object<double>(i_LThigh);
    LTibia = new LibXR::Kinematic::Object<double>(i_LTibia);
    LAnkle = new LibXR::Kinematic::Object<double>(i_LAnkle);
    LAnkle_1 = new LibXR::Kinematic::EndPoint<double>(i_LAnkle_1);

    /* Joints */
    LHipYawPitch = new LibXR::Kinematic::Joint<double>(
        a_LHipYawPitch, NaoRobot::nao_robot->Torso, t_Torso_LHipYawPitch,
        LPelvis, t_LHipYawPitch_LPelvis);
    LHipRoll = new LibXR::Kinematic::Joint<double>(
        a_LHipRoll, LPelvis, t_LPelvis_LHipRoll, LHip, t_LHipRoll_LHip);
    LHipPitch = new LibXR::Kinematic::Joint<double>(
        a_LHipPitch, LHip, t_LHip_LHipPitch, LThigh, t_LHipPitch_LThigh);
    LKneePitch = new LibXR::Kinematic::Joint<double>(
        a_LKneePitch, LThigh, t_LThigh_LKneePitch, LTibia, t_LKneePitch_LTibia);
    LAnklePitch = new LibXR::Kinematic::Joint<double>(
        a_LAnklePitch, LTibia, t_LTibia_LAnklePitch, LAnkle,
        t_LAnklePitch_LAnkle);
    LAnkleRoll = new LibXR::Kinematic::Joint<double>(
        a_LAnkleRoll, LAnkle, t_LAnkle_LAnkleRoll, LAnkle_1,
        t_LAnkleRoll_LAnkle_1);

    LAnkle_1->SetErrorWeight(
        Eigen::Matrix<double, 6, 1>(1.0, 1.0, 1.0, 1.0, 1.0, 1.0));
    LAnkle_1->SetMaxAngularVelocity(20);
    LAnkle_1->SetMaxLineVelocity(50);

    LHipYawPitch->SetBackwardMult(1.0);
    LHipRoll->SetBackwardMult(1.0);
    LHipPitch->SetBackwardMult(1.0);
    LKneePitch->SetBackwardMult(1.0);
    LAnklePitch->SetBackwardMult(1.0);
    LAnkleRoll->SetBackwardMult(1.0);

    void (*thread_func)(LeftLeg *) = [](LeftLeg *l) {
      l->LAnkle_1->SetTargetPosition(
          LibXR::Position(-0.003, 0.0540065, -0.356075));
      l->LAnkle_1->SetTargetQuaternion(LibXR::Quaternion(1., 0., 0., 0.));

      while (true) {
        l->UpdateFeedback();

        NaoRobot::WaitForwardFinish();
        l->LAnkle_1->CalcBackward(1.0, 100, 0.01, 0.2);

        if (NaoRobot::control_enable) {
          l->Control();
        }

        LibXR::Thread::Sleep(1);
      }
    };

    thread.Create<LeftLeg *>(this, thread_func, "LeftLeg", 1024,
                             LibXR::Thread::Priority::MEDIUM);
  }

  void UpdateFeedback() {
    LHipYawPitch->SetState(
        NaoRobot::nao_robot->JointGetPosition(NaoRobot::JointID::LHipYawPitch));
    LHipRoll->SetState(
        NaoRobot::nao_robot->JointGetPosition(NaoRobot::JointID::LHipRoll));
    LHipPitch->SetState(
        NaoRobot::nao_robot->JointGetPosition(NaoRobot::JointID::LHipPitch));
    LKneePitch->SetState(
        NaoRobot::nao_robot->JointGetPosition(NaoRobot::JointID::LKneePitch));
    LAnklePitch->SetState(
        NaoRobot::nao_robot->JointGetPosition(NaoRobot::JointID::LAnklePitch));
    LAnkleRoll->SetState(
        NaoRobot::nao_robot->JointGetPosition(NaoRobot::JointID::LAnkleRoll));
    NaoRobot::FeedbackFinish();
  }

  void Control() {
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::LHipYawPitch,
        LHipYawPitch->runtime_.target_angle.angle());
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::LHipRoll, LHipRoll->runtime_.target_angle.angle());
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::LHipPitch, LHipPitch->runtime_.target_angle.angle());
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::LKneePitch,
        LKneePitch->runtime_.target_angle.angle());
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::LAnklePitch,
        LAnklePitch->runtime_.target_angle.angle());
    NaoRobot::nao_robot->JointPositionControl(
        NaoRobot::JointID::LAnkleRoll,
        LAnkleRoll->runtime_.target_angle.angle());
  }

  /* Inertia */
  LibXR::Inertia<double> i_LPelvis =
      LibXR::Inertia(0.06981, 2.3407e-05, 4.76355e-05, 4.97021e-05, 1.07922e-06,
                     2.76058e-06, -1.76003e-06);
  LibXR::Inertia<double> i_LHip =
      LibXR::Inertia(0.13053, 2.41106e-05, 6.34892e-05, 5.67694e-05,
                     -6.08684e-07, -1.99137e-07, 6.33119e-06);
  LibXR::Inertia<double> i_LThigh =
      LibXR::Inertia(0.38968, 0.000509844, 0.000465358, 0.000301098,
                     2.11296e-06, -7.91029e-06, 5.64129e-05);
  LibXR::Inertia<double> i_LTibia =
      LibXR::Inertia(0.29142, 0.000470586, 0.000412654, 0.000185772,
                     3.60392e-06, 7.13013e-06, -2.86648e-05);
  LibXR::Inertia<double> i_LAnkle =
      LibXR::Inertia(0.13416, 3.22034e-05, 6.7943e-05, 5.48269e-0, -8.83212e-09,
                     0.0, 4.27549e-06);
  LibXR::Inertia<double> i_LAnkle_1 =
      LibXR::Inertia(0.16184, 9.78911e-05, 0.000369977, 0.000419417,
                     7.88039e-06, 1.44233e-06, 6.1279e-06);

  /* Transform */
  LibXR::Transform<double> t_Torso_LHipYawPitch = LibXR::Transform<double>(
      LibXR::Quaternion(1.0, 0.0, 0.0, 0.0), LibXR::Position(0., 0.05, -0.085));
  LibXR::Transform<double> t_LHipYawPitch_LPelvis =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(-0.00781, -0.01114, 0.02661));
  LibXR::Transform<double> t_LPelvis_LHipRoll = LibXR::Transform<double>(
      LibXR::Quaternion(1.0, 0.0, 0.0, 0.0), LibXR::Position(0.0, 0.0, 0.0));
  LibXR::Transform<double> t_LHipRoll_LHip =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(-0.01549, 0.00029, -0.00515));
  LibXR::Transform<double> t_LHip_LHipPitch = LibXR::Transform<double>(
      LibXR::Quaternion(1.0, 0.0, 0.0, 0.0), LibXR::Position(0.0, 0.0, 0.0));
  LibXR::Transform<double> t_LHipPitch_LThigh =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(0.00138, 0.00221, -0.05373));
  LibXR::Transform<double> t_LThigh_LKneePitch = LibXR::Transform<double>(
      LibXR::Quaternion(1.0, 0.0, 0.0, 0.0), LibXR::Position(0.0, 0.0, -0.1));
  LibXR::Transform<double> t_LKneePitch_LTibia =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(0.00453, 0.00225, -0.04936));
  LibXR::Transform<double> t_LTibia_LAnklePitch =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(0.0, 0.0, -0.1029));
  LibXR::Transform<double> t_LAnklePitch_LAnkle =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(0.00045, 0.00029, 0.00685));
  LibXR::Transform<double> t_LAnkle_LAnkleRoll = LibXR::Transform<double>(
      LibXR::Quaternion(1.0, 0.0, 0.0, 0.0), LibXR::Position(0.0, 0.0, 0.0));
  LibXR::Transform<double> t_LAnkleRoll_LAnkle_1 =
      LibXR::Transform<double>(LibXR::Quaternion(1.0, 0.0, 0.0, 0.0),
                               LibXR::Position(0.02542, 0.0033, -0.03239));

  /* Axis */
  LibXR::Axis<double> a_LHipYawPitch =
      LibXR::Axis<double>(0.0, 0.707106, 0.707106);
  LibXR::Axis<double> a_LHipRoll = LibXR::Axis<double>::X();
  LibXR::Axis<double> a_LHipPitch = LibXR::Axis<double>::Y();
  LibXR::Axis<double> a_LKneePitch = LibXR::Axis<double>::Y();
  LibXR::Axis<double> a_LAnklePitch = LibXR::Axis<double>::Y();
  LibXR::Axis<double> a_LAnkleRoll = LibXR::Axis<double>::X();

  /* Chain */
  LibXR::Kinematic::Joint<double> *LHipYawPitch = nullptr;
  LibXR::Kinematic::Object<double> *LPelvis = nullptr;
  LibXR::Kinematic::Joint<double> *LHipRoll = nullptr;
  LibXR::Kinematic::Object<double> *LHip = nullptr;
  LibXR::Kinematic::Joint<double> *LHipPitch = nullptr;
  LibXR::Kinematic::Object<double> *LThigh = nullptr;
  LibXR::Kinematic::Joint<double> *LKneePitch = nullptr;
  LibXR::Kinematic::Object<double> *LTibia = nullptr;
  LibXR::Kinematic::Joint<double> *LAnklePitch = nullptr;
  LibXR::Kinematic::Object<double> *LAnkle = nullptr;
  LibXR::Kinematic::Joint<double> *LAnkleRoll = nullptr;
  LibXR::Kinematic::EndPoint<double> *LAnkle_1 = nullptr;

  /* Thread... */
  LibXR::Thread thread;

  void SetTargetQuaternion(LibXR::Quaternion<double> target) {
    LAnkle_1->SetTargetQuaternion(target);
  }

  void SetTargetPosition(LibXR::Position<double> target) {
    LAnkle_1->SetTargetPosition(target);
  }

  LibXR::Position<double> GetPosition() {
    return LAnkle_1->runtime_.state.translation;
  }

  LibXR::Quaternion<double> GetQuaternion() {
    return LAnkle_1->runtime_.state.rotation;
  }
};
