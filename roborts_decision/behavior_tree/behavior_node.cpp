//
// Created by wpy on 2020/8/23.
//


#include "behavior_node.h"

using namespace roborts_decision;

bool SelectorNode::HasActionOnGoing() {
  return (this->ptr_selector_node_ != nullptr);
}

void SelectorNode::ExitThisAction(){
  this->ptr_selector_node_->Reset();
  this->ptr_selector_node_ = nullptr;
}

std::shared_ptr<AbstractCommonStrategy> SelectorNode::GetAction(){
  if(this->HasActionOnGoing()){
    BehaviorState state = this->ptr_selector_node_->Update();
    if(state != BehaviorState::RUNNING){
      ROS_ERROR("need choose again");
      this->ExitThisAction();
    }
  }
  if (!this->HasActionOnGoing()){
    this->ptr_selector_node_ = this->ChooseActionToExecute();
  }
  return this->ptr_selector_node_;

}
std::shared_ptr<AbstractCommonStrategy> SelectorNode::ChooseActionToExecute() {
  for (auto iter : this->children_node_ptr_vector_){
    if(iter->CanExecuteMe()){
      return iter;
    }
  }
  ROS_ERROR("choose action error");
};

void SelectorNode::AddChildren(std::initializer_list<std::shared_ptr<AbstractCommonStrategy>> child_node_ptr_list){

  for (const auto & child_node_ptr : child_node_ptr_list) {
    children_node_ptr_vector_.push_back(child_node_ptr);
  }
}



BehaviorState SelectorNode::Update() {

  if (children_node_ptr_vector_.empty()) {
    ROS_INFO("children is empty");
    return BehaviorState::SUCCESS;
  }
  const auto action_to_exceed = this->GetAction();
  BehaviorState state = action_to_exceed->Update();
  action_to_exceed->run();
  return state;
}



/**
 * @brief Recover or reset something After getting the result
 * @param state Input behavior state
 */
void SelectorNode::OnTerminate(BehaviorState state){
  switch (state){
    case BehaviorState::IDLE:
      ROS_INFO("%s %s IDLE!", name_.c_str(), __FUNCTION__);
      //TODO: the following recovery measure is called by parent node, and deliver to reset its running child node
      //children_node_ptr_.at(children_node_index_)->Reset();
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
}


/**
 * @brief Set the index of the child node to tick
 * @param children_node_index The expected index of the child node to tick
 */
void SelectorNode::SetChildrenIndex(unsigned int children_node_index){
  children_node_index_=children_node_index;
}



