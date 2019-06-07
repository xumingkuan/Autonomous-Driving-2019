//
// Created by xmk on 5/15/19.
//

#pragma once

#include "homework5/simulation/vehicle_agent.h"

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/math/math_utils.h"
#include "homework5/simulation/vehicle_agent_factory.h"

namespace xumingkuan {

// temporary implementation, similar to sample
class XmkVehicleAgent : public simulation::VehicleAgent {
 public:
  explicit XmkVehicleAgent(const std::string& name) : VehicleAgent(name) {}

  virtual void Initialize(const interface::agent::AgentStatus& /* agent_status */) {
    // Nothing to initialize
  }

  virtual interface::control::ControlCommand RunOneIteration(
      const interface::agent::AgentStatus& agent_status) {
    interface::control::ControlCommand command;
    if (CalcDistance(agent_status.vehicle_status().position(),
                     agent_status.route_status().destination()) < 6.0) {
      position_near_destination_ = true;
    }
    // Vehicle's current position reaches the destination
    if (CalcDistance(agent_status.vehicle_status().position(),
                     agent_status.route_status().destination()) < 3.0) {
      position_reached_destination_ = true;
    }
    // Vehicle's current velocity reaches 5 m/s
    if (CalcVelocity(agent_status.vehicle_status().velocity()) > 5) {
      velocity_reached_threshold_ = true;
    }

    if (position_reached_destination_ || position_near_destination_) {
      if (position_reached_destination_) {
        // Set maximum brake ratio to stop the vehicle
        command.set_brake_ratio(1.0);
      } else {
        command.set_brake_ratio(0.3);
      }
    } else {
      if (!velocity_reached_threshold_) {
        // Set throttle ratio to accelerate
        command.set_throttle_ratio(0.3);
      }
    }
    return command;
  }

 private:
  double CalcDistance(const interface::geometry::Vector3d& position,
                      const interface::geometry::Point3D& destination) {
    double sqr_sum =
        math::Sqr(position.x() - destination.x()) + math::Sqr(position.y() - destination.y());
    return std::sqrt(sqr_sum);
  }

  double CalcVelocity(const interface::geometry::Vector3d& velocity) {
    double sqr_sum = math::Sqr(velocity.x()) + math::Sqr(velocity.y());
    return std::sqrt(sqr_sum);
  }

  // Whether vehicle's current position reaches the destination
  bool position_reached_destination_ = false;
  bool position_near_destination_ = false;
  // Whether vehicle's current velocity reaches 5 m/s
  bool velocity_reached_threshold_ = false;
};

}  // namespace xumingkuan