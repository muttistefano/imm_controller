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

#ifndef IMM_CONTROLLER__FORWARD_CONTROLLERS_BASE_HPP_
#define IMM_CONTROLLER__FORWARD_CONTROLLERS_BASE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "imm_controller_parameters.hpp"

#include "controller_interface/controller_interface.hpp"
#include "imm_controller/visibility_control.h"
#include "rclcpp/subscription.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <boost/scoped_ptr.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/QR>   
#include <eigen3/Eigen/SVD> 


namespace imm_controller
{
using CmdType = geometry_msgs::msg::Twist;

/**
 * \brief Forward command controller for a set of joints and interfaces.
 *
 * This class forwards the command signal down to a set of joints or interfaces.
 *
 * Subscribes to:
 * - \b commands (std_msgs::msg::Float64MultiArray) : The commands to apply.
 */
class ImmController : public controller_interface::ControllerInterface
{
public:
  IMM_CONTROLLER_PUBLIC
  ImmController();

  IMM_CONTROLLER_PUBLIC
  ~ImmController() = default;

  IMM_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  IMM_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  IMM_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  IMM_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  IMM_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  IMM_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  IMM_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  /**
   * Derived controllers have to declare parameters in this method.
   * Error handling does not have to be done. It is done in `on_init`-method of this class.
   */
  void declare_parameters();

  /**
   * Derived controllers have to read parameters in this method and set `command_interface_types_`
   * variable. The variable is then used to propagate the command interface configuration to
   * controller manager. The method is called from `on_configure`-method of this class.
   *
   * It is expected that error handling of exceptions is done.
   *
   * \returns controller_interface::CallbackReturn::SUCCESS if parameters are successfully read and their values are
   * allowed, controller_interface::CallbackReturn::ERROR otherwise.
   */
  controller_interface::CallbackReturn read_parameters();

  std::vector<std::string> joint_names_;
  std::string interface_name_;

  std::vector<std::string> command_interface_types_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;

  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>> _cmd_vel_pub_rt;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_pub_wrapped;

  // _cmd_vel_mm = geometry_msgs::msg::Twist();

  //KDL

  std::string _chain_root;
  std::string _chain_tip;
  KDL::Tree   _kdl_tree;
  KDL::Chain  _kdl_chain_robot;
  KDL::Chain  _kdl_chain_imm;
  KDL::JntArray  _q_robot;
  KDL::Jacobian  _J_robot; 
  KDL::Frame     _fk_robot;
  KDL::Frame     _fk_imm;

  boost::scoped_ptr<KDL::ChainJntToJacSolver> _jnt_to_jac_solver_robot;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> _jnt_to_jac_solver_imm;
  boost::scoped_ptr<KDL::ChainFkSolverPos>    _jnt_to_pose_solver_robot;
  boost::scoped_ptr<KDL::ChainFkSolverPos>    _jnt_to_pose_solver_imm;


  Eigen::Matrix<double,6,1> _base_vel  {0.0,0.0,0.0,0.0,0.0,0.0};
  Eigen::Matrix<double,6,1> _tcp_vel {0.0,0.0,0.0,0.0,0.0,0.0};
  Eigen::Matrix<double,6,1> _q_robot_vel {0.0,0.0,0.0,0.0,0.0,0.0};
  Eigen::Matrix<double,3,1> _q_robot_vel_mm {0.0,0.0,0.0};
  Eigen::Matrix<double,9,1> _q_robot_vel_all {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  Eigen::Matrix<double,6,6> _v_root_tip = Eigen::Matrix<double,6,6>::Zero(6, 6);
  Eigen::Matrix<double,6,6> _v_imm_tip  = Eigen::Matrix<double,6,6>::Zero(6, 6);
  const Eigen::Matrix<double,6,3> _mm_vel {{-1.0,0.0,0.0},
                                           {0.0,-1.0,0.0},
                                           {0.0,0.0,0.0},
                                           {0.0,0.0,0.0},
                                           {0.0,0.0,0.0},
                                           {0.0,0.0,-1.0}};



  Eigen::Matrix<double,6,3> _mm_jac;

  Eigen::Matrix<double,6,9> _jac_complete = Eigen::Matrix<double,6,9>::Zero(6, 9);

  //PARAMS
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

};

 void wrenchMsgToEigen(geometry_msgs::msg::Twist &m, Eigen::Matrix<double,6,1> &e)
 {
   e[0] = m.linear.x;
   e[1] = m.linear.y;
   e[2] = m.linear.z;
   e[3] = m.angular.x;
   e[4] = m.angular.y;
   e[5] = m.angular.z;
 }


  inline Eigen::Matrix3d Skew(double vec[3]) {
    return (Eigen::Matrix3d() << 
        0.0, -vec[2], vec[1],
        vec[2], 0.0, -vec[0],
        -vec[1], vec[0], 0.0).finished();
  }

  inline Eigen::Matrix3d Frame_to_Eigen(double  vec[9]) {
    return (Eigen::Matrix3d() << 
        vec[0], vec[1], vec[2],
        vec[3], vec[4], vec[5],
        vec[6], vec[7], vec[8]).finished();
  }

  void Adjoint_util(Eigen::Matrix<double,6,6> & mat, KDL::Frame frame)
  {
    mat.topLeftCorner(3,3)     = Frame_to_Eigen(frame.M.data);
    mat.bottomRightCorner(3,3) = Frame_to_Eigen(frame.M.data);
    mat.topRightCorner(3,3)    = Skew(frame.p.data) * Frame_to_Eigen(frame.M.data);
  }

  Eigen::Matrix<double,6,6> ja_to_j(double r,double p,double y)
  {
    Eigen::Matrix<double,6,6> mat = Eigen::Matrix<double,6,6>::Zero();
    mat.topLeftCorner(3,3).diagonal() << 1, 1, 1;
    Eigen::Matrix<double,3,3> mat_r;
    mat_r << 1,0,sin(p),0,cos(r),-cos(p)*sin(r),0,sin(r),cos(p)*cos(r);
    mat.bottomRightCorner(3,3) = mat_r.inverse();
    return mat;
  }

void transformKDLToEigenImpl(const KDL::Frame &k, Eigen::Affine3d &e)
  {
    // translation
    for (unsigned int i = 0; i < 3; ++i)
      e(i, 3) = k.p[i];

    // rotation matrix
    for (unsigned int i = 0; i < 9; ++i)
      e(i/3, i%3) = k.M.data[i];

    // "identity" row
    e(3,0) = 0.0;
    e(3,1) = 0.0;
    e(3,2) = 0.0;
    e(3,3) = 1.0;
  }

  inline void spatialDualTranformation(const Eigen::Matrix<double,6,1>& wrench_of_a_in_a, const Eigen::Affine3d& T_b_a, Eigen::Matrix<double,6,1>* wrench_of_b_b)
  {
    (*wrench_of_b_b) << T_b_a.linear()*wrench_of_a_in_a.block(0, 0, 3, 1),
    T_b_a.linear()*wrench_of_a_in_a.block(3, 0, 3, 1) + ((Eigen::Matrix<double,3,1>)(T_b_a.linear()*wrench_of_a_in_a.block(0, 0, 3, 1))).cross(T_b_a.translation());
  }

// template<typename MatType>
// using PseudoInverseType = Eigen::Matrix<typename MatType::Scalar, MatType::ColsAtCompileTime, MatType::RowsAtCompileTime>;

// template<typename MatType>
// PseudoInverseType<MatType> pseudoInverse(const MatType &a, double epsilon = std::numeric_limits<double>::epsilon())
// {
// 	using WorkingMatType = Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, Eigen::Dynamic, 0,
// 																			 MatType::MaxRowsAtCompileTime, MatType::MaxColsAtCompileTime>;
// 	// Eigen::BDCSVD<WorkingMatType> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
//   Eigen::BDCSVD<WorkingMatType> svd(a, Eigen::ComputeFullU | Eigen::ComputeFullV);
// 	svd.setThreshold(epsilon*std::max(a.cols(), a.rows()));
// 	Eigen::Index rank = svd.rank();
// 	Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, MatType::RowsAtCompileTime,
// 								0, Eigen::BDCSVD<WorkingMatType>::MaxDiagSizeAtCompileTime, MatType::MaxRowsAtCompileTime>
// 		tmp = svd.matrixU().leftCols(rank).adjoint();
// 	tmp = svd.singularValues().head(rank).asDiagonal().inverse() * tmp;
// 	return svd.matrixV().leftCols(rank) * tmp;
// }

Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& a, double epsilon = std::numeric_limits<double>::epsilon())
 {
     Eigen::JacobiSVD< Eigen::MatrixXd > svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
     double tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs()(0);
     return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
 }

}  // namespace imm_controller

#endif  // IMM_CONTROLLER__FORWARD_CONTROLLERS_BASE_HPP_

