#pragma once
#include "main.hpp"
#include <behaviortree_cpp/bt_factory.h>

/* 检测摔倒 */
class FallDetectNode : public BT::ConditionNode {
public:
  FallDetectNode(const std::string &name);
  BT::NodeStatus tick() override;
};

/* 移动到球附近 */
class MoveToBallNode : public BT::SyncActionNode {
public:
  MoveToBallNode(const std::string &name);
  BT::NodeStatus tick() override;
};

/* 逼近球 */
class ApproachBallNode : public BT::StatefulActionNode {
public:
  ApproachBallNode(const std::string &name);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

/* 踢球 */
class KickBallNode : public BT::SyncActionNode {
public:
  KickBallNode(const std::string &name);
  BT::NodeStatus tick() override;
};

/* 防守 */
class DefendGoalNode : public BT::SyncActionNode {
public:
  DefendGoalNode(const std::string &name);
  BT::NodeStatus tick() override;
};

/* 移动回初始位置 */
class MoveToHomePositionNode : public BT::SyncActionNode {
public:
  MoveToHomePositionNode(const std::string &name);
  BT::NodeStatus tick() override;
};

/* 检查球是否在己方半场 */
class CheckBallPositionNode : public BT::ConditionNode {
public:
  CheckBallPositionNode(const std::string &name);
  BT::NodeStatus tick() override;
};

/* 等待动作完成 */
class WaitForMotionCompleteNode : public BT::StatefulActionNode {
public:
  WaitForMotionCompleteNode(const std::string &name)
      : BT::StatefulActionNode(name, {}) {}

  BT::NodeStatus onStart() override { return BT::NodeStatus::RUNNING; }

  BT::NodeStatus onRunning() override {
    for (int i = 0; i < 6000; i++) {
      if (!NaoRobot::nao_robot->moving)
        return BT::NodeStatus::SUCCESS;
      usleep(2000); // 2ms 休眠
    }
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override {
    // 这个方法必须实现，哪怕不执行任何操作
    std::cout << "WaitForMotionCompleteNode halted." << std::endl;
  }
};

class BallInOurFieldNode : public BT::ConditionNode {
public:
  BallInOurFieldNode(const std::string &name) : BT::ConditionNode(name, {}) {}

  BT::NodeStatus tick() override {
    return NaoRobot::nao_robot->BallInOurField() ? BT::NodeStatus::SUCCESS
                                                 : BT::NodeStatus::FAILURE;
  }
};

class BallInLeftFieldNode : public BT::ConditionNode {
public:
  BallInLeftFieldNode(const std::string &name) : BT::ConditionNode(name, {}) {}

  BT::NodeStatus tick() override {
    return (NaoRobot::nao_robot->ball_pos.y() < 0.5) ? BT::NodeStatus::SUCCESS
                                                     : BT::NodeStatus::FAILURE;
  }
};

class BallInRightFieldNode : public BT::ConditionNode {
public:
  BallInRightFieldNode(const std::string &name) : BT::ConditionNode(name, {}) {}

  BT::NodeStatus tick() override {
    return (NaoRobot::nao_robot->ball_pos.y() >= 0.5) ? BT::NodeStatus::SUCCESS
                                                      : BT::NodeStatus::FAILURE;
  }
};

#include <behaviortree_cpp/decorator_node.h>

class NegateNode : public BT::DecoratorNode {
public:
  NegateNode(const std::string &name, const BT::NodeConfig &config)
      : BT::DecoratorNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    BT::NodeStatus child_status = child()->executeTick();
    if (child_status == BT::NodeStatus::SUCCESS)
      return BT::NodeStatus::FAILURE;
    if (child_status == BT::NodeStatus::FAILURE)
      return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::RUNNING;
  }
};

#include <behaviortree_cpp/bt_factory.h>

/* 🚀 检查球是否在对面半场 */
class BallInEnemyFieldNode : public BT::ConditionNode {
public:
  BallInEnemyFieldNode(const std::string &name);
  BT::NodeStatus tick() override;
};

class ReturnToHomeInterruptNode : public BT::SyncActionNode {
public:
  ReturnToHomeInterruptNode(const std::string &name);
  BT::NodeStatus tick() override;
};
