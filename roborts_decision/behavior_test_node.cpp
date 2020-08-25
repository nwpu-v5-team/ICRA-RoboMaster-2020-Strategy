//
// Created by kehan on 2020/4/12.
//

#include "behavior_test_node.h"

using namespace roborts_decision;
std::shared_ptr<BehaviorTestNode> BehaviorTestNode::ptr_behavior_test_node = nullptr;

/**
 * Entrypoint of the policy node.
 */
int main(int argc, char** argv) {

  ros::init(argc, argv, "behavior_test_node");
  ros::Time::init();
  ros::Rate loop_rate(data::Hz);

  std::shared_ptr<Blackboard> p_blackboard = Blackboard::GetBlackboard();
  ROS_ERROR("update start");
  //std::shared_ptr<StrategyExecute> p_strategy_execute = StrategyExecute::GetStrategyExecute();

  /**
   *  creat behavior_tree
   */
   std::shared_ptr<BehaviorTestNode> behavior_test_node = BehaviorTestNode::GetBehaviorTestNode();
   std::shared_ptr<BehaviorNode> bt_test = behavior_test_node->CreatRootNode();
  ROS_ERROR("update end");

  while (ros::ok()) {

    p_blackboard->Update();

    bt_test->Run();

    loop_rate.sleep();
  }
}

BehaviorTestNode::BehaviorTestNode() {

}

std::shared_ptr<BehaviorNode> BehaviorTestNode::CreatRootNode(){
  std::shared_ptr<Blackboard> p_blackboard = Blackboard::GetBlackboard();


  /**
   * creat selector_node
   */
  std::shared_ptr<SelectorNode> ptr_selector_node_robot_1_chassis = std::make_shared<SelectorNode>("select robot 1 chissis", p_blackboard);
  std::shared_ptr<MyRobot>p_my_robot_1 =  p_blackboard->GetMyRobot1();
  ptr_selector_node_robot_1_chassis->AddChildren({
      std::make_shared<StopMove>(p_my_robot_1,p_blackboard ),
      std::make_shared<CaptureBulletsMoveStrategy>(p_my_robot_1, p_blackboard),
      std::make_shared<CaptureOurBloodStrategy>(p_my_robot_1, p_blackboard),
      std::make_shared<KeepDirectionAttackMoveStrategy>(p_my_robot_1, p_blackboard),
      std::make_shared<KeepDirectionDefenseMoveStrategy>(p_my_robot_1, p_blackboard)
  });

  std::shared_ptr<SelectorNode> ptr_selector_node_robot_1_gimbal = std::make_shared<SelectorNode>("select robot 1 gimbal", p_blackboard);

  ptr_selector_node_robot_1_gimbal->AddChildren({
      std::make_shared<StopShootStrategy>(p_my_robot_1,p_blackboard),
      std::make_shared<ToBetterOppShootStrategy>(p_my_robot_1,p_blackboard) ,
/*      std::make_shared<ToWorseOppShootStrategy>(p_my_robot_1,p_blackboard)*/
});

  std::shared_ptr<SelectorNode> ptr_selector_node_robot_2_chassis = std::make_shared<SelectorNode>("select robot 2 chissis", p_blackboard);
  std::shared_ptr<MyRobot>p_my_robot_2 =  p_blackboard->GetMyRobot2();
  ptr_selector_node_robot_2_chassis->AddChildren({
                                                     std::make_shared<StopMove>(p_my_robot_2,p_blackboard ),
                                                     std::make_shared<CaptureBulletsMoveStrategy>(p_my_robot_2, p_blackboard),
                                                     std::make_shared<CaptureOurBloodStrategy>(p_my_robot_2, p_blackboard),
                                                     std::make_shared<KeepDirectionAttackMoveStrategy>(p_my_robot_2, p_blackboard),
                                                     std::make_shared<KeepDirectionDefenseMoveStrategy>(p_my_robot_2, p_blackboard)
                                                 });

  std::shared_ptr<SelectorNode> ptr_selector_node_robot_2_gimbal = std::make_shared<SelectorNode>("select robot 2 gimbal", p_blackboard);

  ptr_selector_node_robot_2_gimbal->AddChildren({
                                                    std::make_shared<StopShootStrategy>(p_my_robot_2,p_blackboard),
                                                    std::make_shared<ToBetterOppShootStrategy>(p_my_robot_2,p_blackboard) ,
/*                                                    std::make_shared<ToWorseOppShootStrategy>(p_my_robot_2,p_blackboard)*/
                                                });

  /**
   * creat parallel nodes
   */
   std::shared_ptr<ParallelNode> ptr_parallel_robot_1 = std::make_shared<ParallelNode>("ronot1 parallel node", p_blackboard, 2);
   ptr_parallel_robot_1->AddChildren({
     ptr_selector_node_robot_1_chassis,
     ptr_selector_node_robot_1_gimbal
   });

  std::shared_ptr<ParallelNode> ptr_parallel_robot_2 = std::make_shared<ParallelNode>("ronot2 parallel node", p_blackboard, 2);
  ptr_parallel_robot_2->AddChildren({
                                        ptr_selector_node_robot_2_chassis,
                                        ptr_selector_node_robot_2_gimbal
                                    });


  std::shared_ptr<ParallelNode> root= std::make_shared<ParallelNode>("root parallel node", p_blackboard, 2);
  root->AddChildren({
    ptr_parallel_robot_1,
    ptr_parallel_robot_2
  });
  return root;
}




std::shared_ptr<BehaviorTestNode> BehaviorTestNode::GetBehaviorTestNode() {
  if(BehaviorTestNode::ptr_behavior_test_node == nullptr){
    ptr_behavior_test_node = std::make_shared<BehaviorTestNode>();
  }else{
    // do nothing
  }

  return ptr_behavior_test_node;
}






