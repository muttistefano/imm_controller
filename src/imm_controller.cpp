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
  if (params_.command_joints.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'command_joints' parameter was empty");
    // std::copy(params_.command_joints.begin(), params_.command_joints.end(), std::back_inserter(params_.joints)); 
    return controller_interface::CallbackReturn::ERROR;
  }
  if (params_.state_interface.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'state_interface' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.control_interface.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'control_interface' parameter was empty");
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
  
  if (auto ret = this->read_parameters(); ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  joint_state_interface_.resize(allowed_interface_types_.size());
  joint_command_interface_.resize(allowed_interface_types_.size());

  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { this->rt_command_ptr_.writeFromNonRT(msg); });

  std::string _robot_description_msg = "";
  
  #ifdef TEST_GAZEBO
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "TESTING WITH GAZEBO BUILD " );
    #pragma message("TESTING WITH GAZEBO BUILD ")
    #include<imm_controller/test/ur_urdf.hpp>
    _robot_description_msg = robot_urdf;
  #else
    auto robot_sub = get_node()->create_subscription<std_msgs::msg::String>(
      params_.robot_description_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [&_robot_description_msg,this](std_msgs::msg::String::SharedPtr msg) { _robot_description_msg = msg->data; });

    while(_robot_description_msg == "")
    {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "_robot_description_msg " << _robot_description_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));  
    }
  #endif

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
  
  _error_pub = get_node()->create_publisher<geometry_msgs::msg::Twist>(
    "error_cartesian", rclcpp::SystemDefaultsQoS());
  _fk_pub = get_node()->create_publisher<geometry_msgs::msg::Twist>(
    "fk", rclcpp::SystemDefaultsQoS());
  _tw_pub = get_node()->create_publisher<geometry_msgs::msg::Twist>(
    "twist_cartesian", rclcpp::SystemDefaultsQoS());

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

  // TODO check all
  // has_position_state_interface_ = contains_interface_type(
  //   params_.state_interface, hardware_interface::HW_IF_POSITION);
  // has_velocity_state_interface_ = contains_interface_type(
  //   params_.state_interface, hardware_interface::HW_IF_VELOCITY);
  // has_acceleration_state_interface_ = contains_interface_type(
  //   params_.state_interface, hardware_interface::HW_IF_ACCELERATION);

  // has_position_command_interface_ = contains_interface_type(
  //   params_.control_interface, hardware_interface::HW_IF_POSITION);
  // has_velocity_command_interface_ = contains_interface_type(
  //   params_.control_interface, hardware_interface::HW_IF_VELOCITY);
  // has_acceleration_command_interface_ = contains_interface_type(
  //   params_.control_interface, hardware_interface::HW_IF_ACCELERATION);
  // has_effort_command_interface_ = contains_interface_type(
  //   params_.control_interface, hardware_interface::HW_IF_EFFORT);

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ImmController::command_interface_configuration() const
{
  std::vector<std::string> command_interfaces_config_names;
  for (const auto & interface : params_.control_interface)
  {
    for (const auto & joint : params_.command_joints)
    {
      auto full_name = joint + "/" + interface;
      command_interfaces_config_names.push_back(full_name);
    }
  }

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    command_interfaces_config_names};
}

controller_interface::InterfaceConfiguration ImmController::state_interface_configuration() const
{
  std::vector<std::string> state_interfaces_config_names;
  for (size_t i = 0; i < params_.state_interface.size(); ++i)
  {
    const auto & interface = params_.state_interface[i];
    for (const auto & joint : params_.joints)
    {
      auto full_name = joint + "/" + interface;
      state_interfaces_config_names.push_back(full_name);
    }
  }

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces_config_names};
}


controller_interface::CallbackReturn ImmController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  for (const auto & interface : params_.state_interface)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, params_.joints, interface,
          joint_state_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.", params_.joints.size(),
        interface.c_str(), joint_state_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }
  for (const auto & interface : params_.control_interface)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
          command_interfaces_, params_.command_joints, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' command interfaces, got %zu.", params_.joints.size(),
        interface.c_str(), joint_command_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);


  for (auto index = 0UL; index < params_.joints.size(); ++index)
  {
    _q_robot.data(index) = state_interfaces_[index].get_value();
  }

  if(params_.only_robot)
  {
    _jnt_to_pose_solver_robot->JntToCart(_q_robot,_fk_robot);
    KDLframetoV6(_fk_robot,_twist_integral);
    transformKDLToEigenImpl(_fk_robot,_space_integral);
  }
  else
  {
    _jnt_to_pose_solver_imm->JntToCart(_q_robot, _fk_imm);
    KDLframetoV6(_fk_imm,_twist_integral);
    transformKDLToEigenImpl(_fk_imm,_space_integral);
  }

  RCLCPP_WARN_STREAM(get_node()->get_logger(), "Initial position : \n" << _twist_integral);
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
  auto start = std::chrono::high_resolution_clock::now();
  auto twist_command = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!twist_command || !(*twist_command))
  {
    return controller_interface::return_type::OK;
    RCLCPP_WARN_STREAM(get_node()->get_logger(), "Empty command" << "\n");
    _tcp_vel << 0.0,0.0,0.0,0.0,0.0,0.0;
  }
  else
  {
    imm_controller::wrenchMsgToEigen(*(*twist_command),_tcp_vel);
  }

  // for (const auto & state_interface : state_interfaces_)
  // {
  //   std::string interface_name = state_interface.get_interface_name();
  //   RCLCPP_INFO_STREAM(get_node()->get_logger(), "stat " << state_interface.get_value());
  // }
  //TODO check from joint broadcaster how they do
  for (auto index = 0UL; index < params_.joints.size(); ++index)
  {
    // RCLCPP_INFO_STREAM(get_node()->get_logger(), "stat " << state_interfaces_[index].get_name() << "\n");
    _q_robot.data(index) = state_interfaces_[index].get_value();
  }

  // imm_controller::wrenchMsgToEigen(*(*twist_command),_tcp_vel);

  //ONLY ROBOT ARM
  if(params_.only_robot)
  {
    _jnt_to_jac_solver_robot->JntToJac(_q_robot, _J_robot);
    _jnt_to_pose_solver_robot->JntToCart(_q_robot,_fk_robot);
    Eigen::Affine3d aff_fk_robot;
    transformKDLToEigenImpl(_fk_robot,aff_fk_robot);
    spatialRotation(_tcp_vel,aff_fk_robot.linear(),&_base_vel);
    


    // _space_integral = spatialIntegration(_space_integral,_base_vel,period.seconds());
    Eigen::Affine3d new_int = spatialIntegration(_space_integral,_base_vel,period.seconds());
    _space_integral = new_int;
    _twist_integral << _space_integral.translation(),eig_to_RPY(_space_integral.linear());


    Eigen::Matrix<double,6,1> fkV6;
    KDLframetoV6(_fk_robot,fkV6);
    auto error_cart = cartesian_error(_twist_integral,fkV6);
    
    
    // RCLCPP_INFO_STREAM(get_node()->get_logger(), "_space_integral. eulerAngles \n" << eig_to_RPY(_space_integral.linear()) << "\n");
    // RCLCPP_INFO_STREAM(get_node()->get_logger(), "_space_integral.translation() \n" << _space_integral.translation() << "\n");
    // RCLCPP_INFO_STREAM(get_node()->get_logger(), "fkV6 \n" << fkV6 << "\n");
    // RCLCPP_INFO_STREAM(get_node()->get_logger(), "_base_vel \n" << _base_vel << "\n");
    // RCLCPP_INFO_STREAM(get_node()->get_logger(), "_twist_integral \n" << _twist_integral << "\n");
    

    geometry_msgs::msg::Twist err_msg;
    err_msg.linear.x = error_cart(0);
    err_msg.linear.y = error_cart(1);
    err_msg.linear.z = error_cart(2);
    err_msg.angular.x = error_cart(3);
    err_msg.angular.y = error_cart(4);
    err_msg.angular.z = error_cart(5);
    _error_pub->publish(err_msg);
    geometry_msgs::msg::Twist tw_msg;
    tw_msg.linear.x  = _twist_integral(0);
    tw_msg.linear.y  = _twist_integral(1);
    tw_msg.linear.z  = _twist_integral(2);
    tw_msg.angular.x = _twist_integral(3);
    tw_msg.angular.y = _twist_integral(4);
    tw_msg.angular.z = _twist_integral(5);
    _tw_pub->publish(tw_msg);
    geometry_msgs::msg::Twist fk_msg;
    fk_msg.linear.x  = fkV6(0);
    fk_msg.linear.y  = fkV6(1);
    fk_msg.linear.z  = fkV6(2);
    fk_msg.angular.x = fkV6(3);
    fk_msg.angular.y = fkV6(4);
    fk_msg.angular.z = fkV6(5);
    _fk_pub->publish(fk_msg);

    Eigen::Matrix< double, 6, 6> KK = Eigen::Matrix< double, 6, 6>::Zero();
    KK.diagonal() << 0.3, 0.3, 0.3, 0.3 , 0.3 , 0.3;
    // _q_robot_vel =  _J_robot.data.inverse() * (_base_vel + KK * error_cart);
    // _q_robot_vel =  ((_J_robot.data.inverse() * (error_cart ))  * 2) + (_J_robot.data.inverse() * _base_vel);
    _q_robot_vel =  (_J_robot.data.inverse() * _base_vel);
    

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
  spatialRotation(_tcp_vel,aff_fk_imm.linear(),&_base_vel);

  
  // Eigen::Affine3d new_int = spatialIntegration(_space_integral,_base_vel,period.seconds());
  Eigen::Affine3d new_int = spatialIntegration(aff_fk_imm,_base_vel,period.seconds());
  _space_integral = new_int;
  _twist_integral << _space_integral.translation(),eig_to_RPY(_space_integral.linear());


  Eigen::Matrix<double,6,1> fkV6;
  KDLframetoV6(_fk_imm,fkV6);
  auto error_cart = cartesian_error(_twist_integral,fkV6);

  geometry_msgs::msg::Twist err_msg;
  err_msg.linear.x = error_cart(0);
  err_msg.linear.y = error_cart(1);
  err_msg.linear.z = error_cart(2);
  err_msg.angular.x = error_cart(3);
  err_msg.angular.y = error_cart(4);
  err_msg.angular.z = error_cart(5);
  _error_pub->publish(err_msg);
  geometry_msgs::msg::Twist tw_msg;
  tw_msg.linear.x  = _twist_integral(0);
  tw_msg.linear.y  = _twist_integral(1);
  tw_msg.linear.z  = _twist_integral(2);
  tw_msg.angular.x = _twist_integral(3);
  tw_msg.angular.y = _twist_integral(4);
  tw_msg.angular.z = _twist_integral(5);
  _tw_pub->publish(tw_msg);
  geometry_msgs::msg::Twist fk_msg;
  fk_msg.linear.x  = fkV6(0);
  fk_msg.linear.y  = fkV6(1);
  fk_msg.linear.z  = fkV6(2);
  fk_msg.angular.x = fkV6(3);
  fk_msg.angular.y = fkV6(4);
  fk_msg.angular.z = fkV6(5);
  _fk_pub->publish(fk_msg);


  if(params_.omni)
  {
    _jac_complete << _J_robot.data,_mm_vel_omni;
  }
  else
  {
    _jac_complete << _J_robot.data,_mm_vel_diff;
  }

  auto jac_inv = pseudoInverse(_jac_complete);
  Eigen::Matrix< double, 6, 6> KK = Eigen::Matrix< double, 6, 6>::Zero();
  KK.diagonal() << 0.5, 0.5, 0.5, 0.5 , 0.5 , 0.5;
  // _q_robot_vel_all =  jac_inv * (_base_vel + KK * error_cart);
  // _q_robot_vel_all =  jac_inv * _base_vel ;

  Eigen::Matrix<double, 6, 1> goal {-M_PI/2.0,-M_PI/2.0,-M_PI/2.0,-M_PI/2.0,M_PI/2.0,-M_PI/2.0};

  // _q_robot_vel_all =  jac_inv * (_base_vel + KK * error_cart);
  Eigen::Matrix<double, 9, 1> q_err;
  q_err << goal-_q_robot.data,0.0,0.0,0.0;
  _q_robot_vel_all =  jac_inv * _base_vel + 1 * (Eigen::Matrix<double, 9, 9>::Identity() - jac_inv * _jac_complete )*q_err ;
  // _q_robot_vel_all =  jac_inv * (_base_vel + KK * error_cart);// + 0.3 * (Eigen::Matrix<double, 9, 9>::Identity() - jac_inv * _jac_complete )*q_err ;

  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "q_err \n" << q_err << "\n");
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "PD \n" << 0.3 * (Eigen::Matrix<double, 9, 9>::Identity() - jac_inv * _jac_complete )*q_err << "\n");
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "_q_robot_vel_all \n" << _q_robot_vel_all << "\n");

  _q_robot_vel = _q_robot_vel_all.head(6);

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
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "duration \n" << duration.count() << "\n");
  return controller_interface::return_type::OK;
}

}  // namespace imm_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  imm_controller::ImmController, controller_interface::ControllerInterface)
