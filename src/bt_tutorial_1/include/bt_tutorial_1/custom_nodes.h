#pragma once
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h> 

class ApproachObject : public BT::SyncActionNode
{
public:
  ApproachObject(const std::string& name) :
      BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    std::cout << "Approaching object:" << this->name()<< std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

// Action that prints a message and returns SUCCESS
class CheckBattery : public BT::SyncActionNode
{
public:
  CheckBattery(const std::string& name) :
      BT::SyncActionNode(name, {})
  {
  }


  BT::NodeStatus tick() override
  {
    std::cout << "Checking battery" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};


class GripperInterface
{
public:
  GripperInterface(): _open(true) {}
    
  BT::NodeStatus open() 
  {
    _open = true;
    std::cout << "GripperInterface::open" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus close() 
  {
    std::cout << "GripperInterface::close" << std::endl;
    _open = false;
    return BT::NodeStatus::SUCCESS;
  }

private:
  bool _open; // shared information
};

// Action that prints a message and returns SUCCESS
class OpenGripper : public BT::SyncActionNode
{
public:
  OpenGripper(const std::string& name) :
      BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    std::cout << "Opening gripper" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};







class SaySomething : public BT::SyncActionNode
{
  public:
  SaySomething(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}
};


