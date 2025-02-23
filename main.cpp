#include "behavior_nodes.hpp"
#include "sdl.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <cstring>
#include <iostream>
#include <map>
#include <unistd.h>

NaoRobot *NaoRobot::nao_robot = nullptr;
Sim2D *Sim2D::sim2d = nullptr;

std::map<RobotType, std::string> behavior_tree_files = {
    {Defender1, "../behavior_tree/bt_defender1.xml"},
    {Defender2, "../behavior_tree/bt_defender2.xml"},
    {Attacker, "../behavior_tree/bt_attacker.xml"},
    {GoalKeeper, "../behavior_tree/bt_goalkeeper.xml"}};

int main(int argc, char **argv) {
  RobotType robot_type = Defender1;
  double gate_addr = GATE_1_X;

  if (argc > 2) {
    std::map<std::string, RobotType> robot_mapping = {
        {"defender1", Defender1},
        {"defender2", Defender2},
        {"attacker", Attacker},
        {"goalkeeper", GoalKeeper}};

    if (robot_mapping.count(argv[1]))
      robot_type = robot_mapping[argv[1]];
    if (strcmp(argv[2], "1") == 0)
      gate_addr = GATE_1_X;
    else if (strcmp(argv[2], "2") == 0)
      gate_addr = GATE_2_X;
  }

  NaoRobot nao_robot(robot_type, gate_addr);
  LibXR::PlatformInit(&nao_robot.supervisor);
  int index = robot_type;

  if (gate_addr == GATE_2_X) {
    index += 4;
  }
  Sim2D sim2d(index);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<FallDetectNode>("FallDetect");
  factory.registerNodeType<MoveToBallNode>("MoveToBall");
  factory.registerNodeType<ApproachBallNode>("ApproachBall");
  factory.registerNodeType<KickBallNode>("KickBall");
  factory.registerNodeType<DefendGoalNode>("DefendGoal");
  factory.registerNodeType<MoveToHomePositionNode>("MoveToHomePosition");
  factory.registerNodeType<WaitForMotionCompleteNode>("WaitForMotionComplete");
  factory.registerNodeType<BallInOurFieldNode>("BallInOurField");
  factory.registerNodeType<BallInLeftFieldNode>("BallInLeftField");
  factory.registerNodeType<BallInRightFieldNode>("BallInRightField");
  factory.registerNodeType<BallInEnemyFieldNode>("BallInEnemyField");
  factory.registerNodeType<ReturnToHomeInterruptNode>("ReturnToHomeInterrupt");
  factory.registerNodeType<NegateNode>("Negate");

  auto tree = factory.createTreeFromFile(behavior_tree_files[robot_type]);

  while (true) {
    tree.tickOnce();
    LibXR::Thread::Sleep(20);
  }

  return 0;
}
