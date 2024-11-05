/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/World.hh>

namespace astrobee_gazebo {

class WorldPluginSpeed : 
 public gz::sim::System,
 public gz::sim::ISystemConfigure
{
  public: void Configure(
                const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventManager) override
  {
    world_.reset( new gz::sim::World(_entity));
 
    // Query the simulation speed
    double simulation_speed = 1.0;

    auto sdf_clone = _sdf->Clone();

    if (sdf_clone->HasElement("simulation_speed"))
      simulation_speed = sdf_clone->GetElement("simulation_speed")->Get<float>();

    simulation_speed *= 125;
    // Set the simulation speed
    //gzmsg << "Setting target update rate to " << simulation_speed << std::endl;
    //physics::PhysicsEnginePtr engine = world->Physics();
    //engine->SetRealTimeUpdateRate(simulation_speed);
    
  }


 protected:
  std::shared_ptr<gz::sim::World> world_;

}; // class WorldPluginSpeed

}   // namespace astrobee_gazebo

GZ_ADD_PLUGIN(
  astrobee_gazebo::WorldPluginSpeed,
  gz::sim::System,
  astrobee_gazebo::WorldPluginSpeed::ISystemConfigure
)
