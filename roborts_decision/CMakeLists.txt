project(roborts_decision)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    nav_msgs
    geometry_msgs
    actionlib
    roborts_common
    roborts_msgs
    roborts_costmap
    dynamic_reconfigure
    )

find_package(Eigen3 REQUIRED)
find_package(ProtoBuf REQUIRED)
find_package(PCL REQUIRED COMPONENT common io)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

# For clion symbol updates
include_directories(
    ../../../devel/include
)

add_definitions(${PCL_DEFINITIONS})

#generate proto files
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")

file(GLOB DecisionProtoSrc proto/*.cc)
file(GLOB DecisionProtoHds proto/*.h)

if(NOT DecisionProtoSrc)
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
    DecisionProtoSrc
    DecisionProtoHds
    ${ProtoFiles}
    )
endif()

generate_dynamic_reconfigure_options(
    ../config/roborts_dynamic_cfg.cfg
)

catkin_package()


set(BLACKBOARD_SRC
    blackboard/blackboard_common.h
    blackboard/blackboard_raw.h
    blackboard/blackboard_raw.cpp
    blackboard/my_robot.h
    blackboard/my_robot.cpp
    blackboard/enemy_robot.cpp
    blackboard/enemy_robot.h
    blackboard/blackboard.cpp
    blackboard/blackboard.h
    blackboard/prediction_system.cpp
    blackboard/prediction_system.h
    blackboard/field.cpp
    blackboard/field.h)

set(BEHAVIOR_TREE
    behavior_tree/behavior_state.h
    behavior_tree/behavior_tree.h
    behavior_tree/behavior_node.cpp
    behavior_tree/behavior_node.h)


set(ROBOTSTRATEGY_SRC
    strategy/abstract_common_strategy.h
    strategy/shoot_strategys/stop_shoot_strategy.cpp
    strategy/shoot_strategys/stop_shoot_strategy.h
    strategy/shoot_strategys/toBetterOpp_shoot_strategy.cpp
    strategy/shoot_strategys/toBetterOpp_shoot_strategy.h
    strategy/shoot_strategys/toWorseOpp_shoot_strategy.cpp
    strategy/shoot_strategys/toWorseOpp_shoot_strategy.h
    strategy/move_strategys/captureHomeBlood_move_strategy.cpp
    strategy/move_strategys/captureHomeBlood_move_strategy.h
    strategy/move_strategys/captureEnemyBlood_move_strategy.cpp
    strategy/move_strategys/captureEnemyBlood_move_strategy.h
    strategy/move_strategys/captureBullets_move_strategy.cpp
    strategy/move_strategys/captureBullets_move_strategy.h
    strategy/move_strategys/keepDirection_attack_move_strategy.cpp
    strategy/move_strategys/keepDirection_attack_move_strategy.h
    strategy/move_strategys/keepDirection_defense_move_strategy.cpp
    strategy/move_strategys/keepDirection_defense_move_strategy.h
    strategy/move_strategys/stop_move_strategy.cpp
    strategy/move_strategys/stop_move_strategy.h
    strategy/strategy_execute.cpp
    strategy/strategy_execute.h
    strategy/move_strategys/explore_move_strategy.cpp
    strategy/move_strategys/explore_move_strategy.h)

add_executable(behavior_test_node
    ${DecisionProtoHds}
    ${DecisionProtoSrc}
    ${BLACKBOARD_SRC}
    ${ROBOTSTRATEGY_SRC}
    ${BEHAVIOR_TREE}
    executor/chassis_executor.h
    executor/chassis_executor.cpp
    executor/gimbal_executor.h
    executor/gimbal_executor.cpp
    executor/gimbal_control.h
    executor/gimbal_control.cpp
    ../roborts_common/include/interface/dynamic_reconfigure_interface.h
    ../roborts_common/include/interface/dynamic_reconfigure_interface.cpp
        behavior_test_node.cpp
        behavior_test_node.h)

target_link_libraries(behavior_test_node
    PRIVATE
    roborts_costmap
    ${catkin_LIBRARIES}
    ${PROTOBUF_LIBRARIES})
    #libblackboard)

add_dependencies(behavior_test_node
    roborts_msgs_generate_messages
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})