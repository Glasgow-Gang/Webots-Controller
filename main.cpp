#include "main.hpp"

#include <Eigen/Core>
#include <cstdio>

#include "inertial.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

/* Utility function */
double clamp(double value, double min, double max) {
  return (value < min) ? min : (value > max) ? max : value;
}

int main() {
  auto head_intertial =
      Inertial::toInertial3D(0.000957766, 0.000816836, 0.000984976, 8.78814e-06,
                             5.33702e-06, -2.99579e-05);
  Eigen::Vector3d head_position(-0.0, 0, 0.05258);

  Inertial head_inertial(0.60533, head_intertial, head_position);

  Eigen::Vector3d head_pitch_position(0.0, 0.0, 0.0);
  Eigen::Vector3d head_pitch_axis(0, 1.0, 0);

  head_inertial.Display();
  auto head_pitch_inertial = head_inertial.Transform(head_pitch_position);
  head_pitch_inertial.Display();
  std::cout << "Head pitch inertia: "
            << head_pitch_inertial.GetInertiaAxis(head_pitch_axis) << std::endl;

  NaoRobot nao;
  while (nao.step(nao.timeStep) != -1) {
  }
  return 0;
}
