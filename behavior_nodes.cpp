#include "behavior_nodes.hpp"
#include <iostream>
#include <unistd.h>

/* 🚀 检测摔倒 */
FallDetectNode::FallDetectNode(const std::string &name)
    : BT::ConditionNode(name, {}) {}

BT::NodeStatus FallDetectNode::tick() {
  auto acc = NaoRobot::nao_robot->accelerometer_data;
  if (std::abs(acc[0]) > std::abs(acc[1]) &&
      std::abs(acc[0]) > std::abs(acc[2])) {
    if (acc[0] < -5.0) {
      NaoRobot::nao_robot->PlayMotion(MotionFile::StandUpFromFront);
    } else if (acc[2] > 0) {
      NaoRobot::nao_robot->PlayMotion(MotionFile::StandUpFromBack);
    }
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

/* 🚀 移动到球附近 */
MoveToBallNode::MoveToBallNode(const std::string &name)
    : BT::SyncActionNode(name, {}) {}

BT::NodeStatus MoveToBallNode::tick() {
  bool success = NaoRobot::nao_robot->RobotMoveAround(
      NaoRobot::nao_robot->ball_pos.x(), NaoRobot::nao_robot->ball_pos.y(),
      0.15, NaoRobot::nao_robot->BallGetAngleToEnemyGate(), 2, 0.04);

  return success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

/* 🚀 逼近球 */
ApproachBallNode::ApproachBallNode(const std::string &name)
    : BT::StatefulActionNode(name, {}) {}

BT::NodeStatus ApproachBallNode::onStart() { return BT::NodeStatus::RUNNING; }

BT::NodeStatus ApproachBallNode::onRunning() {
  double distance = NaoRobot::nao_robot->RobotGetDistanceTo(
      NaoRobot::nao_robot->ball_pos.x(), NaoRobot::nao_robot->ball_pos.y());

  if (distance < 0.03) {
    return BT::NodeStatus::SUCCESS;
  } else {
    NaoRobot::nao_robot->RobotGoto(NaoRobot::nao_robot->ball_pos.x(),
                                   NaoRobot::nao_robot->ball_pos.y());
    return BT::NodeStatus::RUNNING;
  }
}

void ApproachBallNode::onHalted() { NaoRobot::nao_robot->Stop(); }

/* 🚀 踢球 */
KickBallNode::KickBallNode(const std::string &name)
    : BT::SyncActionNode(name, {}) {}

BT::NodeStatus KickBallNode::tick() {
  NaoRobot::nao_robot->PlayMotion(LongPass);
  std::cout << "LongPass\n";

  for (int i = 0; i < 6000; i++) {
    if (NaoRobot::nao_robot->PlayMotion(LongPass))
      break;
    usleep(2000); // 2ms 休眠
  }

  return BT::NodeStatus::SUCCESS;
}

/* 🚀 防守 */
DefendGoalNode::DefendGoalNode(const std::string &name)
    : BT::SyncActionNode(name, {}) {}

BT::NodeStatus DefendGoalNode::tick() {
  float y = NaoRobot::nao_robot->ball_pos.y();
  y = std::clamp<double>(y, 0.35, 0.65);

  bool success = NaoRobot::nao_robot->RobotGoto(
      NaoRobot::nao_robot->GetOwnGoalKeeperInitX(), y, 0.15);

  return success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

/* 🚀 移动回初始位置 */
MoveToHomePositionNode::MoveToHomePositionNode(const std::string &name)
    : BT::SyncActionNode(name, {}) {}

BT::NodeStatus MoveToHomePositionNode::tick() {
  RobotType role = NaoRobot::nao_robot->robot_type;
  bool success = false;

  if (role == Defender1) {
    success = NaoRobot::nao_robot->RobotGoto(
        NaoRobot::nao_robot->GetOwnDefenderInitX(),
        NaoRobot::nao_robot->GetOwnDefenderInitY(), 0.15);
  } else if (role == Attacker) {
    success = NaoRobot::nao_robot->RobotGoto(
        NaoRobot::nao_robot->GetOwnAttackerInitX(),
        NaoRobot::nao_robot->GetOwnAttackerInitY(), 0.15);
  } else if (role == GoalKeeper) {
    success = NaoRobot::nao_robot->RobotGoto(
        NaoRobot::nao_robot->GetOwnGoalKeeperInitX(),
        std::clamp(NaoRobot::nao_robot->ball_pos.y(), 0.35, 0.65), 0.15);
  }

  return success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

/* 🚀 检查球是否在己方半场 */
CheckBallPositionNode::CheckBallPositionNode(const std::string &name)
    : BT::ConditionNode(name, {}) {}

BT::NodeStatus CheckBallPositionNode::tick() {
  return NaoRobot::nao_robot->BallInOurField() ? BT::NodeStatus::SUCCESS
                                               : BT::NodeStatus::FAILURE;
}

/* 🚀 检查球是否在对面半场 */
BallInEnemyFieldNode::BallInEnemyFieldNode(const std::string &name)
    : BT::ConditionNode(name, {}) {}

BT::NodeStatus BallInEnemyFieldNode::tick() {
  double ball_x = NaoRobot::nao_robot->ball_pos.x();
  double field_center_x = 0.0; // 假设场地中心 x 坐标为 0
  return (ball_x > field_center_x) ? BT::NodeStatus::SUCCESS
                                   : BT::NodeStatus::FAILURE;
}

/* 🚀 强制回原点并中断当前动作 */
ReturnToHomeInterruptNode::ReturnToHomeInterruptNode(const std::string &name)
    : BT::SyncActionNode(name, {}) {}

BT::NodeStatus ReturnToHomeInterruptNode::tick() {
  NaoRobot::nao_robot->Stop(); // 立即停止当前动作
  MoveToHomePositionNode moveToHome("MoveToHome");
  return moveToHome.tick(); // 调用原 `MoveToHomePositionNode` 逻辑
}