#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include "Eigen/src/Geometry/AngleAxis.h"
#include "Eigen/src/Geometry/Quaternion.h"
#include "inertia.hpp"
#include "list.hpp"
#include "transform.hpp"
#include <iostream>

namespace LibXR {

namespace Kinematic {
template <typename Scalar = LIBXR_DEFAULT_SCALAR> class Joint;
template <typename Scalar = LIBXR_DEFAULT_SCALAR> class Object;
template <typename Scalar = LIBXR_DEFAULT_SCALAR> class EndPoint;
template <typename Scalar = LIBXR_DEFAULT_SCALAR> class StartPoint;

template <typename Scalar> class Joint {
public:
  typedef struct Param {
    Transform<Scalar> parent2this;
    Transform<Scalar> this2child;
    Axis<Scalar> axis;
  } Param;

  typedef struct Runtime {
    Eigen::AngleAxis<Scalar> state_angle;
    Eigen::AngleAxis<Scalar> target_angle;

    Eigen::Matrix<Scalar, 3, 3> inertia;

    Axis<Scalar> state_axis;
    Axis<Scalar> target_axis;

    Transform<Scalar> state;
    Transform<Scalar> target;
  } Runtime;

  Param param_;
  Runtime runtime_;

  Object<Scalar> *parent = nullptr;
  Object<Scalar> *child = nullptr;

  Joint(Axis<Scalar> axis, Object<Scalar> *parent,
        Transform<Scalar> &parent2this, Object<Scalar> *child,
        Transform<Scalar> &this2child)
      : parent(parent), child(child), param_({parent2this, this2child, axis}) {
    runtime_.inertia = Eigen::Matrix<Scalar, 3, 3>::Zero();
    auto link = new typename Object<Scalar>::Link(this);
    parent->joints.Add(*link);
    child->parent = this;
  }

  void SetState(Scalar state) {
    if (state > M_PI) {
      state -= 2 * M_PI;
    }
    if (state < -M_PI) {
      state += 2 * M_PI;
    }
    runtime_.state_angle.angle() = state;
    runtime_.state_angle.axis() = param_.axis;
  }

  void SetTarget(Scalar target) {
    if (target > M_PI) {
      target -= 2 * M_PI;
    }
    if (target < -M_PI) {
      target += 2 * M_PI;
    }
    runtime_.target_angle.angle() = target;
    runtime_.target_angle.axis() = param_.axis;
  }
};

template <typename Scalar> class Object {
public:
  typedef List::Node<Joint<Scalar> *> Link;

  typedef struct {
    Inertia<Scalar> inertia;
  } Param;

  typedef struct {
    Transform<Scalar> state;
    Transform<Scalar> target;
  } Runtime;

  List joints;

  Joint<Scalar> *parent = nullptr;

  Param param_;
  Runtime runtime_;

  Object(Inertia<Scalar> &inertia) : param_({inertia}) {}

  void SetPosition(const Position<Scalar> &pos) { runtime_.state = pos; }

  void SetQuaternion(const Quaternion<Scalar> &quat) { runtime_.state = quat; }

  void CalcBackward() {}
};

template <typename Scalar> class EndPoint : public Object<Scalar> {
private:
  Eigen::Matrix<Scalar, 6, Eigen::Dynamic> *J = nullptr;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> *delta_theta = nullptr;
  int joint_num = 0;

public:
  EndPoint(Inertia<Scalar> &inertia) : Object<Scalar>(inertia) {}

  Quaternion<Scalar> target_quat_;
  Position<Scalar> target_pos_;

  void SetTargetQuaternion(const Quaternion<Scalar> &quat) {
    target_quat_ = quat;
  }

  void SetTargetPosition(const Position<Scalar> &pos) { target_pos_ = pos; }

  Eigen::Matrix<Scalar, 6, 1> CalcBackward(int max_step = 10,
                                           Scalar max_err = 1e-3,
                                           Scalar step_size = 1.0) {
    Eigen::Matrix<Scalar, 6, 1> error;

    /* Initialize */
    if (J == nullptr) {
      Object<Scalar> *tmp = this;
      for (int i = 0;; i++) {
        if (tmp->parent == nullptr) {
          joint_num = i;
          break;
        }
        tmp = tmp->parent->parent;
      }

      J = new Eigen::Matrix<Scalar, 6, Eigen::Dynamic>(6, joint_num);

      delta_theta = new Eigen::Matrix<Scalar, Eigen::Dynamic, 1>(joint_num);
    }

    for (int step = 0; step < max_step; ++step) {
      // 1. 计算当前的误差

      error.template head<3>() =
          target_pos_ - this->runtime_.target.translation;
      error.template tail<3>() =
          (Eigen::Quaternion<Scalar>(this->runtime_.target.rotation)
               .conjugate() *
           target_quat_)
              .vec();

      auto err_norm = error.norm();

      if (err_norm < max_err) {
        return error;
      }

      // 2. 计算雅可比矩阵
      do {
        Joint<Scalar> *joint = this->parent;
        for (int joint_index = 0; joint_index < joint_num; joint_index++) {
          // 使用每个关节对位姿的影响计算雅可比矩阵列
          Eigen::Matrix<Scalar, 6, 1> d_transform;
          d_transform.template head<3>() = joint->runtime_.target_axis.cross(
              this->runtime_.target.translation -
              joint->runtime_.target.translation);

          d_transform.template tail<3>() = joint->runtime_.target_axis;

          J->col(joint_index) = d_transform;

          joint = joint->parent->parent;
        }
      } while (0);

      // 3. 计算角度增量：delta_theta = J^+ * error，其中J^+是雅可比矩阵的伪逆
      *delta_theta = J->completeOrthogonalDecomposition().pseudoInverse() *
                     error * step_size / std::sqrt(err_norm);

      // 4. 更新关节角度
      do {
        Joint<Scalar> *joint = this->parent;

        for (int joint_index = 0; joint_index < joint_num; joint_index++) {

          Eigen::AngleAxis<Scalar> target_angle_axis_delta(
              (*delta_theta)(joint_index), joint->runtime_.target_axis);

          target_angle_axis_delta =
              Quaternion<Scalar>(
                  Eigen::Quaternion<Scalar>(target_angle_axis_delta)) /
              joint->runtime_.target.rotation;

          joint->SetTarget(joint->runtime_.target_angle.angle() +
                           (*delta_theta)(joint_index));

          joint = joint->parent->parent;
        }
      } while (0);

      // 5. 重新计算前向运动学
      do {
        Joint<Scalar> *joint = this->parent;

        for (int joint_index = 0; joint_index < joint_num; joint_index++) {
          if (joint_index == joint_num - 1) {
            auto start_point =
                reinterpret_cast<StartPoint<Scalar> *>(joint->parent);
            start_point->CalcTargetForward();
            break;
          }

          joint = joint->parent->parent;
        }
      } while (0);
    }

    return error;
  }
};

template <typename Scalar> class StartPoint : public Object<Scalar> {
public:
  StartPoint(Inertia<Scalar> &inertia) : Object<Scalar>(inertia) {}

  void CalcForward() {
    this->runtime_.target = this->runtime_.state;
    this->joints.Foreach(_ForwardForeachFun, *this);
  }

  void CalcTargetForward() {
    this->runtime_.target = this->runtime_.state;
    this->joints.Foreach(_TargetForwardForeachFun, *this);
  }

  void CalcInertia() {
    Joint<Scalar> *res = nullptr;
    this->joints.Foreach(_InertiaForeachFunStart, res);
  }

  static ErrorCode _InertiaForeachFunStart(Joint<Scalar> *&joint,
                                           Joint<Scalar> *&parent) {
    UNUSED(parent);
    joint->runtime_.inertia =
        joint->child->param_.inertia
            .Translate(joint->runtime_.state.translation -
                       joint->child->runtime_.state.translation)
            .Rotate(joint->child->runtime_.state.rotation /
                    joint->runtime_.state.rotation);

    joint->runtime_.inertia = Inertia<Scalar>::Rotate(
        joint->runtime_.inertia,
        Eigen::Quaternion<Scalar>(joint->runtime_.state_angle));

    joint->child->joints.Foreach(_InertiaForeachFun, joint);

    joint->child->joints.Foreach(_InertiaForeachFunStart, joint);

    return ErrorCode::OK;
  }

  static ErrorCode _InertiaForeachFun(Joint<Scalar> *&joint,
                                      Joint<Scalar> *&parent) {

    auto new_inertia = joint->child->param_.inertia
                           .Translate(parent->runtime_.state.translation -
                                      joint->child->runtime_.state.translation)
                           .Rotate(parent->runtime_.state.rotation /
                                   joint->runtime_.state.rotation);

    parent->runtime_.inertia = new_inertia + parent->runtime_.inertia;

    joint->child->joints.Foreach(_InertiaForeachFun, parent);

    return ErrorCode::OK;
  }

  static ErrorCode _ForwardForeachFun(Joint<Scalar> *&joint,
                                      StartPoint<Scalar> &start) {
    Transform<Scalar> t_joint(joint->parent->runtime_.state +
                              joint->param_.parent2this);

    joint->runtime_.state = t_joint;

    Transform<Scalar> t_child(joint->runtime_.state.rotation *
                                  joint->runtime_.state_angle,
                              joint->runtime_.state.translation);

    t_child = t_child + joint->param_.this2child;

    joint->child->runtime_.state = t_child;
    joint->runtime_.state_axis =
        joint->runtime_.state.rotation * joint->param_.axis;
    joint->runtime_.target_axis = joint->runtime_.state_axis;

    joint->runtime_.target = joint->runtime_.state;
    joint->runtime_.target_angle = joint->runtime_.state_angle;
    joint->child->runtime_.target = joint->child->runtime_.state;

    joint->child->joints.Foreach(_ForwardForeachFun, start);

    return ErrorCode::OK;
  }

  static ErrorCode _TargetForwardForeachFun(Joint<Scalar> *&joint,
                                            StartPoint<Scalar> &start) {
    Transform<Scalar> t_joint(joint->parent->runtime_.target +
                              joint->param_.parent2this);

    joint->runtime_.target = t_joint;

    Transform<Scalar> t_child(joint->runtime_.target.rotation *
                                  joint->runtime_.target_angle,
                              joint->runtime_.target.translation);

    t_child = t_child + joint->param_.this2child;
    joint->child->runtime_.target = t_child;

    joint->runtime_.target_axis =
        joint->runtime_.target.rotation * joint->param_.axis;

    joint->child->joints.Foreach(_TargetForwardForeachFun, start);

    return ErrorCode::OK;
  }
};

} // namespace Kinematic

} // namespace LibXR