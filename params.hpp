#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include "inertial.hpp"

const double g = 9.81;

/* ------------ Head ------------ */
const double m_head = 0.60533;
const double I_head_pitch = 0.00249037;
const double L_head_pitch = 0.0526;
/* K Matrix */
Eigen::Matrix<double, 3, 6> K_head =
    (Eigen::Matrix<double, 3, 6>() << 0.691, 0, 0, 0.04084, 0, 0, 0, 0.3518, 0,
     0, 0.03, 0, 0, 0, 0, 0, 0, 0)
        .finished();
