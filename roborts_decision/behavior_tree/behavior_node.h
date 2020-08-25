/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBORTS_DECISION_BEHAVIOR_NODE_H
#define ROBORTS_DECISION_BEHAVIOR_NODE_H

#include <chrono>
#include <thread>
#include <vector>

#include "../blackboard/blackboard_raw.h"
#include "behavior_state.h"
#include "../strategy/strategy_execute.h"

namespace roborts_decision{
/**
 * @brief Type of behavior tree node
 */
enum class BehaviorType {
  PARALLEL,       ///<Parallel Composite Node
  SELECTOR,       ///<Selector Composite Node
  SEQUENCE,       ///<Sequence Composite Node
  ACTION,         ///<Action Node
  PRECONDITION,   ///<Precondition Node
};
/**
 * @brif Abort Type of behavior tree precondition node
 * @details For more information refer to https://docs.unrealengine.com/en-us/Engine/AI/BehaviorTrees/NodeReference/Decorators
 */
/*enum class AbortType {
  NONE,           ///<Do not abort anything
  SELF,           ///<Abort self, and any sub-trees running under this node
  LOW_PRIORITY,   ///<Abort any nodes to the right of this node
  BOTH            ///<Abort self, any sub-trees running under me, and any nodes to the right of this node
};*/
/**
 * @brief Behavior tree base node
 */
class BehaviorNode : public std::enable_shared_from_this<BehaviorNode>{
 public:

  typedef std::shared_ptr<BehaviorNode> Ptr;
  BehaviorNode(){}
  /**
   * @brief Constructor of BehaviorNode
   * @param name Name of the behavior node
   * @param behavior_type Type of the behavior node
   * @param blackboard_ptr Blackboard of the behavior node
   */
  BehaviorNode(std::string name, BehaviorType behavior_type, const Blackboard::Ptr &blackboard_ptr):
      name_(name),
      behavior_type_(behavior_type),
      blackboard_ptr_(blackboard_ptr),
      behavior_state_(BehaviorState::IDLE),
      level_(0){}

  virtual ~BehaviorNode()= default;
  /**
   * @brief Run the behavior node
   * @details Roughly including 3 process
   *          1. OnInitilaize: Initialize or reset some variables, when tick the node that is not running.
   *          2. Update: Update and feedback the behavior state.
   *          3. OnTerminate: Reset or recover after getting the result.
   * @return Behavior state
   */
  BehaviorState Run(){

    if (behavior_state_ != BehaviorState::RUNNING) {
      OnInitialize();
    }

    behavior_state_ = Update();

    if (behavior_state_ != BehaviorState::RUNNING) {
      OnTerminate(behavior_state_);
    }

    return behavior_state_;
  }
  /**
   * @brief Reset the behavior node
   * @details manually invoke the IDLE terminate function when the node is in running status
   */
  virtual void Reset(){
    if (behavior_state_ == BehaviorState::RUNNING){
      behavior_state_ = BehaviorState::IDLE;
      OnTerminate(behavior_state_);
    }
  }
  /**
   * @brief Get the type of the behavior node
   * @return The type of the behavior node
   */
  BehaviorType GetBehaviorType(){
    return behavior_type_;
  }
  /**
   * @brief Get the state of the behavior node
   * @return The state of the behavior node
   */
  BehaviorState GetBehaviorState(){
    return behavior_state_;
  }
  /**
   * @brief Get the name of the behavior node
   * @return The name of the behavior node
   */
  std::string GetName(){
    return name_.c_str();
  }
  /**
   * @brief Set the parent of the behavior node
   * @param parent_node_ptr
   */
  void SetParent(BehaviorNode::Ptr  parent_node_ptr){
    parent_node_ptr_ = parent_node_ptr;
  }
  /**
   * @brief Get the parent node of the behavior node
   * @return The parent node of the behavior node
   */
  BehaviorNode::Ptr GetParent(){
    return parent_node_ptr_ ;
  }
  /**
   * @brief Get the child node of the behavior node
   * @return The child node of the behavior node, for the base class it always returns nullptr
   */
  virtual BehaviorNode::Ptr GetChild(){
    return nullptr;
  }
  /**
   * @brief Get the level of the behavior node
   * @return  The level of the behavior node
   */
  unsigned int GetLevel(){
    return level_;
  }
  /**
   * @brief Set the level of the behavior node
   * @param level The expected level of the behavior node
   */
  void SetLevel(unsigned int level){
    level_ = level;
  }
 protected:
  /**
   * @brief Tick the node and update the state of the behavior node
   * @return the state of the behavior node
   */
  virtual BehaviorState Update() = 0;
  /**
   * @brief Initialize something before starting to tick the node
   */
  virtual void OnInitialize() = 0;
  /**
   * @brief Recover or reset something After getting the result
   * @param state Input behavior state
   */
  virtual void OnTerminate(BehaviorState state) = 0;

  //! Node name
  std::string name_;
  //! State
//  std::mutex behavior_state_mutex_;
  BehaviorState behavior_state_;
  //! Type
  BehaviorType behavior_type_;
  //! Blackboard
  Blackboard::Ptr blackboard_ptr_;
  //! Parent Node Pointer
  BehaviorNode::Ptr parent_node_ptr_;
  //! Level of the tree
  unsigned  int level_;
};

/**
 * @brief Behavior tree composite node inherited from BehaviorNode
 */
class CompositeNode: public BehaviorNode{
 public:
  /**
   * @brief Constructor of CompositeNode
   * @param name Name of the behavior node
   * @param behavior_type Type of the behavior node
   * @param blackboard_ptr Blackboard of the behavior node
   */
  CompositeNode(std::string name, BehaviorType behavior_type, const Blackboard::Ptr &blackboard_ptr):
      BehaviorNode::BehaviorNode(name, behavior_type, blackboard_ptr),
      children_node_index_(0) {
  }

  virtual ~CompositeNode()= default;
  /**
   * @brief Add child behavior node to the composite node
   * @param child_node_ptr The expected child behavior node
   */
  virtual void AddChildren(const BehaviorNode::Ptr& child_node_ptr){
    children_node_ptr_.push_back(child_node_ptr);
    child_node_ptr->SetParent(shared_from_this());
    child_node_ptr->SetLevel(level_+1);
  }
  /**
   * @brief Add a list of child behavior nodes to the composite node
   * @param child_node_ptr A list of the expected child behavior nodes
   */
  virtual void AddChildren(std::initializer_list<BehaviorNode::Ptr> child_node_ptr_list){
    for (auto child_node_ptr = child_node_ptr_list.begin(); child_node_ptr!=child_node_ptr_list.end(); child_node_ptr++) {
      children_node_ptr_.push_back(*child_node_ptr);
      (*child_node_ptr)->SetParent(shared_from_this());
      (*child_node_ptr)->SetLevel(level_+1);
    }
  }
  /**
   * @brief Get the child behavior node that it is turn to tick
   * @return The child behavior node
   */
  virtual BehaviorNode::Ptr GetChild(){
    return children_node_ptr_.at(children_node_index_) ;
  }
  /**
   * @brief Get the list of child behavior nodes
   * @return The list of child behavior nodes
   */
  std::vector<BehaviorNode::Ptr>& GetChildren(){
    return children_node_ptr_;
  }
  /**
   * @brief Get the index of the child behavior node that it is turn to tick
   * @return The index of the child behavior node that it is turn to tick
   */
  unsigned int GetChildrenIndex(){
    return children_node_index_;
  }
  /**
   * @brief Get the number of child behavior nodes
   * @return The number of child behavior nodes
   */
  unsigned int GetChildrenNum(){
    return children_node_ptr_.size();
  }

 protected:
  /**
   * @brief Tick the node and update the state of the behavior node
   * @return the state of the behavior node
   */
  virtual BehaviorState Update() = 0;
  /**
   * @brief Initialize something before starting to tick the node
   */
  virtual void OnInitialize() = 0;
  /**
   * @brief Recover or reset something After getting the result
   * @param state Input behavior state
   */
  virtual void OnTerminate(BehaviorState state) = 0;
  //! the list of child nodes
  std::vector<BehaviorNode::Ptr> children_node_ptr_;
  //! the index of child nodes
  unsigned int children_node_index_;

};
/**
 * @brief Behavior tree selector node inherited from CompositeNode
 */
class SelectorNode: public CompositeNode{
 public:
  /**
   * @brief Constructor of SelectorNode
   * @param name Name of the behavior node
   * @param blackboard_ptr Blackboard of the behavior node
   */
  SelectorNode(std::string name, const Blackboard::Ptr &blackboard_ptr):
      CompositeNode::CompositeNode(name, BehaviorType::SELECTOR, blackboard_ptr) {
    this->ptr_selector_node_ = nullptr;
  }
  virtual ~SelectorNode() = default;


   virtual void AddChildren(std::initializer_list<std::shared_ptr<AbstractCommonStrategy>> child_node_ptr_list);

 protected:
  void SetChildrenIndex(unsigned int children_node_index);

  /**
   * @brief Initialize something before starting to tick the node
   */
  virtual void OnInitialize(){
    children_node_index_ = 0;
    ROS_INFO("%s %s", name_.c_str(), __FUNCTION__);
  }

  virtual BehaviorState Update();

  std::shared_ptr<AbstractCommonStrategy> GetAction();

  bool HasActionOnGoing();

  void ExitThisAction();

  std::shared_ptr<AbstractCommonStrategy> ChooseActionToExecute();

  void OnTerminate(BehaviorState state);
 private:
  std::shared_ptr<AbstractCommonStrategy> ptr_selector_node_;
  std::vector<std::shared_ptr<AbstractCommonStrategy>> children_node_ptr_vector_;


};



/**
 * @brief Behavior tree parallel node inherited from CompositeNode
 */
class ParallelNode: public CompositeNode{
 public:
  /**
   * @brief Constructor of ParallelNode
   * @param name Name of the behavior node
   * @param blackboard_ptr Blackboard of the behavior node
   * @param threshold Threshold number of child nodes to determine behavior state of success
   */
  ParallelNode(std::string name, const Blackboard::Ptr &blackboard_ptr,
               unsigned int threshold):
      CompositeNode::CompositeNode(name, BehaviorType::PARALLEL, blackboard_ptr),
      threshold_(threshold),
      success_count_(0),
      failure_count_(0){

  }

  virtual ~ParallelNode() = default;
 protected:
  /**
   * @brief Initialize something before starting to tick the node
   * @details Initialize and reset the success and failure count of each child node
   */
  virtual void OnInitialize(){
    failure_count_=0;
    success_count_=0;
    children_node_done_.clear();
    children_node_done_.resize(children_node_ptr_.size(),false);
    ROS_INFO("initialize start");
    ROS_INFO("%s %s", name_.c_str(), __FUNCTION__);
  }
  /**
   * @brief Tick the node and update the state of the behavior node
   * @return the state of the behavior node
   */
  virtual BehaviorState Update(){

    if (children_node_ptr_.empty()) {
      return BehaviorState::SUCCESS;
    }

    for (unsigned int index = 0; index!=children_node_ptr_.size(); index++) {
      if (!children_node_done_.at(index)){
        BehaviorState state = children_node_ptr_.at(index)->Run();

/*        if (state == BehaviorState::SUCCESS) {
          children_node_done_.at(index) = true;
          if (++success_count_ >= threshold_) {
            return BehaviorState::SUCCESS;
          }
        }
        else if (state == BehaviorState::FAILURE) {
          children_node_done_.at(index) = true;
          if (++failure_count_ >= children_node_ptr_.size()-threshold_) {
            return BehaviorState::FAILURE;
          }
        }*/

      }
    }
    return BehaviorState::RUNNING;
  }
  /**
   * @brief Recover or reset something After getting the result
   * @param state Input behavior state
   */
  virtual void OnTerminate(BehaviorState state){
    switch (state){
      case BehaviorState::IDLE:
        ROS_INFO("%s %s IDLE!", name_.c_str(), __FUNCTION__);
        break;
      case BehaviorState::SUCCESS:
        ROS_INFO("%s %s SUCCESS!", name_.c_str(), __FUNCTION__);
        break;
      case BehaviorState::FAILURE:
        ROS_INFO("%s %s FAILURE!", name_.c_str(), __FUNCTION__);
        break;
      default:
        ROS_INFO("%s %s ERROR!", name_.c_str(), __FUNCTION__);
        return;
    }
    //TODO: no matter what state, the node would reset all running children to terminate.
    for (unsigned int index = 0; index!=children_node_ptr_.size(); index++) {
      children_node_ptr_.at(index)->Reset();
    }
  }

  //! a list of result checker flags for each child node
  std::vector<bool> children_node_done_;
  //! the current number of child nodes with success behavior state
  unsigned int success_count_;
  //! the current number of child nodes with failure behavior state
  unsigned int failure_count_;
  //! the number of child nodes with success behavior state to determine success of the parallel node
  unsigned int threshold_;
};


} //namespace roborts_decision


#endif //ROBORTS_DECISION_BEHAVIOR_NODE_H
