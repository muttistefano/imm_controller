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

#ifndef IMM_CONTROLLER__MULTI_INTERFACE_IMM_CONTROLLER_HPP_
#define IMM_CONTROLLER__MULTI_INTERFACE_IMM_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "imm_controller/imm_base.hpp"
#include "imm_controller/visibility_control.h"
#include "multi_interface_imm_controller_parameters.hpp"

namespace imm_controller
{
/**
 * \brief Multi interface forward command controller for a set of interfaces.
 *
 * This class forwards the command signal down to a set of interfaces on the specified joint.
 *
 * \param joint Name of the joint to control.
 * \param interface_names Names of the interfaces to command.
 *
 * Subscribes to:
 * - \b commands (std_msgs::msg::Float64MultiArray) : The commands to apply.
 */
class MultiInterfaceForwardCommandController
: public imm_controller::ForwardControllersBase
{
public:
  IMM_CONTROLLER_PUBLIC
  MultiInterfaceForwardCommandController();

protected:
  void declare_parameters() override;
  controller_interface::CallbackReturn read_parameters() override;

  using Params = multi_interface_imm_controller::Params;
  using ParamListener = multi_interface_imm_controller::ParamListener;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
};

}  // namespace imm_controller

#endif  // IMM_CONTROLLER__MULTI_INTERFACE_IMM_CONTROLLER_HPP_
