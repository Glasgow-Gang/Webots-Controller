#include <Eigen/Core>
#include <iostream>

class Inertial {
 public:
  static Eigen::Matrix3d toInertial3D(double ixx, double iyy, double izz,
                                      double ixy, double iyz, double ixz) {
    Eigen::Matrix3d i;
    i << ixx, -ixy, -ixz, -ixy, iyy, -iyz, -ixz, -iyz, izz;
    return i;
  }

  Inertial(double mass, Eigen::Matrix3d i, Eigen::Vector3d position)
      : mass_(mass), i_(i), position_(position) {}

  void Display() {
    std::cout << "Inertial: " << std::endl
              << i_ << std::endl
              << "Position: " << std::endl
              << position_ << std::endl
              << "Mass: " << std::endl
              << mass_ << std::endl;
  }

  Inertial Transform(const Eigen::Vector3d& position) {
    auto d = position - position_;
    return Inertial(
        mass_,
        i_ + mass_ * (d.squaredNorm() * Eigen::Matrix3d::Identity() -
                      d * d.transpose()),
        position);
  }

  double GetInertiaAxis(Eigen::Vector3d axis) { return axis.dot(i_ * axis); }

  double mass_;
  Eigen::Matrix3d i_;
  Eigen::Vector3d position_;
};