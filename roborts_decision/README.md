# The strategy module for ICRA RoboMaster 2020 {#mainpage}


## The bottom actions submodule

The main entrypoint is main(int argc, char** argv).
Two objects are constructed when started:
- roborts_decision::Blackboard contains all information that blackboard should contain.
- roborts_decision::StrategyExecute receives the message dqn_network sends and execute
  the corresponding strategy.

## The strategy selection submodule

Action selection are written in module dqn_network, using Python and Tensorflow.

## directory structure

- **camke_module**, **config**, **data**: Config files.
- **proto**: Messages to communicate to other ROS nodes.
- **behavior_tree**: Strategy selection using the behavior tree model.
- **dqn_decision**: Strategy selection using the Deep Q Learning.
- **blackboard**: Exposes the roborts_decision::StrategyExecute class, which provides all information.
- **strategy**: Code to execute the chosen strategy.
- **executor**: Provides low-level API that move the robots.