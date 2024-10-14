#include "main.hpp"

void NaoRobot::HeadControl() {
  spdlog::info("# HeadControl");
  double pitch = JointGetPosition(NaoRobot::JointID::HeadPitch);
  double yaw = JointGetPosition(NaoRobot::JointID::HeadYaw);
  spdlog::debug("Joint Pitch = {}", pitch);
  spdlog::debug("Joint Yaw = {}", yaw);
  spdlog::debug(
      "\nK = \n[{}, {}, {}, {}, {}, {}] \n[{}, {}, {}, {}, {}, {}] \n[{}, {}, "
      "{}, {}, {}, {}]",
      K_head(0, 0), K_head(0, 1), K_head(0, 2), K_head(0, 3), K_head(0, 4),
      K_head(0, 5), K_head(1, 0), K_head(1, 1), K_head(1, 2), K_head(1, 3),
      K_head(1, 4), K_head(1, 5), K_head(2, 0), K_head(2, 1), K_head(2, 2),
      K_head(2, 3), K_head(2, 4), K_head(2, 5));

  double psi_beta = body_posture_.yaw;      // Yaw angle of the body
  double theta_beta = body_posture_.pitch;  // Pitch angle of the body
  double phi_beta = body_posture_.roll;     // Roll angle of the body

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

  // Calculate the target rotation matrix C_gamma^alpha for the head
  Eigen::Matrix3d C_gamma_alpha_target;
  double theta_gamma =
      target_position_[static_cast<int>(NaoRobot::JointID::HeadPitch)];
  double psi_gamma =
      target_position_[static_cast<int>(NaoRobot::JointID::HeadYaw)];
  double phi_gamma = phi_beta;
  C_gamma_alpha_target << cos(theta_gamma) * cos(psi_gamma),
      cos(theta_gamma) * sin(psi_gamma), -sin(theta_gamma),
      sin(phi_gamma) * sin(theta_gamma) * cos(psi_gamma) -
          cos(phi_gamma) * sin(psi_gamma),
      sin(phi_gamma) * sin(theta_gamma) * sin(psi_gamma) +
          cos(phi_gamma) * cos(psi_gamma),
      sin(phi_gamma) * cos(theta_gamma),
      cos(phi_gamma) * sin(theta_gamma) * cos(psi_gamma) +
          sin(phi_gamma) * sin(psi_gamma),
      cos(phi_gamma) * sin(theta_gamma) * sin(psi_gamma) -
          sin(phi_gamma) * cos(psi_gamma),
      cos(phi_gamma) * cos(theta_gamma);

  Eigen::Matrix3d C_gamma_beta_target =
      C_beta_alpha.transpose() * C_gamma_alpha_target;

  double target_yaw =
      atan2(C_gamma_beta_target(1, 0), C_gamma_beta_target(0, 0));
  double target_pitch =
      atan2(-C_gamma_beta_target(2, 0),
            sqrt(C_gamma_beta_target(2, 2) * C_gamma_beta_target(2, 2) +
                 C_gamma_beta_target(2, 1) * C_gamma_beta_target(2, 1)));
  double target_roll =
      atan2(C_gamma_beta_target(2, 1), C_gamma_beta_target(2, 2));

  spdlog::debug("Target yaw: {}", psi_gamma);
  spdlog::debug("Target pitch: {}", theta_gamma);
  spdlog::debug("Target roll: {}", phi_gamma);
  spdlog::debug("Target joint yaw: {}", target_yaw);
  spdlog::debug("Target joint pitch: {}", target_pitch);
  spdlog::debug("Target joint roll: {}", target_roll);

  double theta_yaw =
      JointGetPosition(NaoRobot::JointID::HeadYaw);  // Yaw angle of the head
  double theta_pitch = JointGetPosition(
      NaoRobot::JointID::HeadPitch);  // Pitch angle of the head
  double theta_roll = 0;              // Roll angle of the head

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
  theta_gamma = atan2(-r_31, sqrt(r_32 * r_32 + r_33 * r_33));  // Pitch 角
  // double theta_beta_calculated = atan2(r_32, r_33);   // Roll 角

  static Eigen::Matrix<double, 6, 1> old_x =
      (Eigen::Matrix<double, 6, 1>() << pitch, yaw, 0, 0, 0, 0).finished();

  Eigen::Matrix<double, 6, 1> x;
  x << pitch + target_pitch, yaw + target_yaw, 0,
      (pitch + target_pitch - old_x(0, 0)) / timeStep * 1000,
      (yaw + target_yaw - old_x(1, 0)) / timeStep * 1000, 0;

  old_x = x;

  spdlog::debug("\nx =\n [{}, {}, {}, {}, {}, {}]", x(0, 0), x(1, 0), x(2, 0),
                x(3, 0), x(4, 0), x(5, 0));

  Eigen::Matrix<double, 3, 1> u_k = K_head * x;
  Eigen::Matrix<double, 3, 1> u_ff =
      (Eigen::Matrix<double, 3, 1>()
           << m_head * L_head_pitch * g * sin(theta_gamma),
       0, 0)
          .finished();
  spdlog::debug("\nu_k =\n [{}, {}, {}]", u_k(0, 0), u_k(1, 0), u_k(2, 0));
  spdlog::debug("\nu_ff =\n [{}, {}, {}]", u_ff(0, 0), u_ff(1, 0), u_ff(2, 0));

  Eigen::Matrix<double, 3, 1> u = -u_k + u_ff;
  spdlog::debug("\nu =\n [{}, {}, {}]", u(0, 0), u(1, 0), u(2, 0));

  JointTorqueControl(NaoRobot::JointID::HeadPitch, u(0, 0));
  JointTorqueControl(NaoRobot::JointID::HeadYaw, u(1, 0));
  // JointTorqueControl(NaoRobot::JointID::HeadRoll, u(2, 0));
}
