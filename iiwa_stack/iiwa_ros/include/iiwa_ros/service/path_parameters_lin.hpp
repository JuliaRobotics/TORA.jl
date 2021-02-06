/**
 * Copyright (C) 2019 Salvatore Virga - salvo.virga@tum.de
 * Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <iiwa_msgs/SetSmartServoLinSpeedLimits.h>
#include <iiwa_ros/service/iiwa_services.hpp>

namespace iiwa_ros {
namespace service {

/**
 * @brief This class provides a wrapper for the PathParametersLinService service.
 * Once an object of this class is initialized using the appropriate robot namespace name,
 * it is possible to call its functions to set the desired max cartesian velocity to be applied during linear motions.
 */
class PathParametersLinService : public iiwaServices<iiwa_msgs::SetSmartServoLinSpeedLimits> {
public:
  PathParametersLinService() = default;
  virtual ~PathParametersLinService() override = default;

  virtual void init(const std::string& robot_namespace) override;

  /**
   * @brief Set the maximum cartesian velocity to be applied during SmartServoLin motions.
   *
   * @param [in] max_cartesian_velocity - Maximum velocity in its Cartesian components.
   * @return bool - success status.
   */
  bool setMaxCartesianVelocity(const geometry_msgs::Twist max_cartesian_velocity);

protected:
  virtual bool callService() override;
};

}  // namespace service
}  // namespace iiwa_ros
