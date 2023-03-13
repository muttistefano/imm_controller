// Copyright 2021 Stogl Robotics Consulting UG (haftungsbescrhänkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "imm_controller/imm_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace imm_controller
{
ImmController::ImmController()
: controller_interface::ControllerInterface(),
  rt_command_ptr_(nullptr),
  joints_command_subscriber_(nullptr)
{
}

void ImmController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn ImmController::read_parameters()
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if (params_.joints.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.interface_name.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_name' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.robot_chain_root.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'robot_chain_root' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.robot_chain_tip.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'robot_chain_tip' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.robot_description_topic.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'robot_description_topic' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.amr_base_link.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'amr_base_link' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // if (params_.only_robot.empty())
  // {
  //   RCLCPP_ERROR(get_node()->get_logger(), "'only_robot' parameter was empty");
  //   return controller_interface::CallbackReturn::ERROR;
  // }

  for (const auto & joint : params_.joints)
  {
    command_interface_types_.push_back(joint + "/" + params_.interface_name);
  }



  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImmController::on_init()
{
  try
  {
    declare_parameters();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }  

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImmController::on_configure( const rclcpp_lifecycle::State & /*previous_state*/)
{
  
  if (auto ret = this->read_parameters(), ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { this->rt_command_ptr_.writeFromNonRT(msg); });


  std::string _robot_description_msg = "";
  
  auto robot_sub = get_node()->create_subscription<std_msgs::msg::String>(
    params_.robot_description_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [&_robot_description_msg,this](std_msgs::msg::String::SharedPtr msg) { _robot_description_msg = msg->data; });

  while(_robot_description_msg == "")
  {
    // RCLCPP_ERROR_STREAM(get_node()->get_logger(), "_robot_description_msg " << _robot_description_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));  
  }

  kdl_parser::treeFromString(_robot_description_msg,_kdl_tree);
  _kdl_tree.getChain(params_.robot_chain_root,params_.robot_chain_tip,_kdl_chain_robot);
  _kdl_tree.getChain(params_.amr_base_link,params_.robot_chain_tip,_kdl_chain_imm);
  _jnt_to_jac_solver_robot.reset(new KDL::ChainJntToJacSolver(_kdl_chain_robot));
  _jnt_to_jac_solver_imm.reset(new KDL::ChainJntToJacSolver(_kdl_chain_imm));
  _jnt_to_pose_solver_robot.reset(new KDL::ChainFkSolverPos_recursive(_kdl_chain_robot));
  _jnt_to_pose_solver_imm.reset(new KDL::ChainFkSolverPos_recursive(_kdl_chain_imm));

  _q_robot.resize(_kdl_chain_robot.getNrOfJoints());
  _J_robot.resize(_kdl_chain_robot.getNrOfJoints());

  RCLCPP_WARN_STREAM(get_node()->get_logger(), "robotic joints " << _kdl_chain_robot.getNrOfJoints());

  _cmd_vel_pub_wrapped = get_node()->create_publisher<geometry_msgs::msg::Twist>(
    params_.cmd_vel_topic, rclcpp::SystemDefaultsQoS());

  _cmd_vel_pub_rt =
    std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(_cmd_vel_pub_wrapped);

  // Initialize state message
  _cmd_vel_pub_rt->lock();
  _cmd_vel_pub_rt->msg_ = geometry_msgs::msg::Twist();
  _cmd_vel_pub_rt->unlock();

  if(params_.omni)
  {
    _jac_complete.resize(6,9);
    _jac_complete = Eigen::Matrix<double,6,9>::Zero(6, 9);
    _q_robot_vel_all.resize(9,1);
    _q_robot_vel_all << 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;
  }
  else
  {
    _jac_complete.resize(6,8);
    _jac_complete = Eigen::Matrix<double,6,8>::Zero(6, 8);
    _q_robot_vel_all.resize(8,1);
    _q_robot_vel_all << 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0;

  }

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ImmController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_types_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration ImmController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  //TODO separate ?
  state_interfaces_config.names = command_interface_types_;

  return state_interfaces_config;
}


controller_interface::CallbackReturn ImmController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  // ATTENTION(destogl): Shouldn't we use ordered interface all the time?
  //TODO state_interfaces_types instead of command
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    ordered_interfaces;
  if (
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, command_interface_types_, std::string(""), ordered_interfaces) ||
    command_interface_types_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_types_.size(), ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    state_ordered_interfaces;
  if (
    !controller_interface::get_ordered_interfaces(
          state_interfaces_, params_.joints, "position",
          state_ordered_interfaces))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_types_.size(), state_ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImmController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ImmController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  auto twist_command = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!twist_command || !(*twist_command))
  {
    return controller_interface::return_type::OK;
  }

  // for (const auto & state_interface : state_interfaces_)
  // {
  //   std::string interface_name = state_interface.get_interface_name();
  //   RCLCPP_INFO_STREAM(get_node()->get_logger(), "stat " << state_interface.get_value());
  // }
  //TODO check from joint broadcaster how they do
  for (auto index = 0UL; index < state_interfaces_.size(); ++index)
  {
    // RCLCPP_INFO_STREAM(get_node()->get_logger(), "stat " << state_interfaces_[index].get_name() << "\n");
    _q_robot.data(index) = state_interfaces_[index].get_value();
  }
  
  imm_controller::wrenchMsgToEigen(*(*twist_command),_tcp_vel);

  if(params_.only_robot)
  {
    _jnt_to_jac_solver_robot->JntToJac(_q_robot, _J_robot);
    _jnt_to_pose_solver_robot->JntToCart(_q_robot,_fk_robot);
    Eigen::Affine3d aff_fk_robot;
    transformKDLToEigenImpl(_fk_robot,aff_fk_robot);
    spatialDualTranformation(_tcp_vel, aff_fk_robot, &_base_vel);

    _q_robot_vel =  _J_robot.data.inverse() * _base_vel;

    for (auto index = 0UL; index < command_interfaces_.size(); ++index)
    {
      command_interfaces_[index].set_value(command_interfaces_[index].get_value() + (period.seconds() * _q_robot_vel(index)));
    }
    return controller_interface::return_type::OK;
  }

  _jnt_to_jac_solver_imm->JntToJac(_q_robot, _J_robot);
  _jnt_to_pose_solver_imm->JntToCart(_q_robot, _fk_imm);

  Eigen::Affine3d aff_fk_imm;
  transformKDLToEigenImpl(_fk_imm,aff_fk_imm);
  spatialDualTranformation(_tcp_vel, aff_fk_imm, &_base_vel);

  
  if(params_.omni)
  {
    _jac_complete << _J_robot.data,_mm_vel_omni;
  }
  else
  {
    _jac_complete << _J_robot.data,_mm_vel_diff;
  }

  auto jac_inv = pseudoInverse(_jac_complete);
  _q_robot_vel_all =  jac_inv * _base_vel;

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "_jac_complete \n" << _jac_complete << "\n");
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "_mm_vel " << _mm_vel << "\n");
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "_q_robot_vel_all " << _q_robot_vel_all << "\n");

  _q_robot_vel = _q_robot_vel_all.head(6);
  // _q_robot_vel_mm = _q_robot_vel_all.tail<3>();

  for (auto index = 0UL; index < command_interfaces_.size(); ++index)
  {
    command_interfaces_[index].set_value(command_interfaces_[index].get_value() + (period.seconds() * _q_robot_vel(index)));
  }

  if (_cmd_vel_pub_rt->trylock())
  {
    auto & msg = _cmd_vel_pub_rt->msg_;
    _cmd_vel_pub_rt->msg_ = geometry_msgs::msg::Twist();
    if(params_.omni)
    {
      msg.linear.x  = _q_robot_vel_all(6);
      msg.linear.y  = _q_robot_vel_all(7);
      msg.angular.z = _q_robot_vel_all(8);
    }
    else
    {
      msg.linear.x  = _q_robot_vel_all(6);
      msg.angular.z = _q_robot_vel_all(7);
    }

    _cmd_vel_pub_rt->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace imm_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  imm_controller::ImmController, controller_interface::ControllerInterface)
