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

// Gazebo includes
#include <astrobee_gazebo/astrobee_gazebo.h>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>

namespace astrobee_gazebo {

// This class is a plugin that calls the GNC autocode to predict
// the forced to be applied to the rigid body
class GazeboModelPluginDrag : public FreeFlyerModelPlugin {
 public:
  GazeboModelPluginDrag() : FreeFlyerModelPlugin("gazebo_drag", ""),
    coefficient_(1.05), area_(0.092903), density_(1.225) {}

  ~GazeboModelPluginDrag() {
  }

 protected:
  // Called when the plugin is loaded into the simulator
  void LoadCallback(NodeHandle &nh, gz::sim::EntityComponentManager &_ecm) {
    // Drag coefficient
    if (sdf_->HasElement("coefficient"))
      coefficient_ = sdf_->Get<double>("coefficient");
    // Cross-sectional area
    if (sdf_->HasElement("area"))
      area_ = sdf_->Get<double>("area");
    // Air density
    if (sdf_->HasElement("density"))
      density_ = sdf_->Get<double>("density");
  }

  // Called on simulation reset
  void Reset() {
  }

  // Called on each sensor update event  
  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) 
  {
    // Calculate drag
    //drag_ = GetLink()->RelativeLinearVel(); // ANA HACK - Check if this works
    std::optional<gz::math::Vector3d> linvel = GetLink()->WorldLinearVelocity(_ecm, gz::math::Vector3d(0, 0, 0));
    if(!linvel)
      return;
    
    drag_ = linvel.value();
    
    vel_ = drag_.Length();
    drag_ = -0.5 * coefficient_ * area_ * density_
           * vel_ * vel_ * drag_.Normalize();

    // Apply the force and torque to the model
    GetLink()->AddWorldForce(_ecm, drag_, gz::math::Vector3d(0,0,0) ); //AddRelativeForce(drag_); // ANA HACK - Check if this work

  }

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) 
  {
  }                


 private:
  double coefficient_, area_, density_, vel_;              // Drag parameters
  gz::math::Vector3d drag_;
};

}   // namespace astrobee_gazebo


// Register this plugin with the simulator
GZ_ADD_PLUGIN(
  astrobee_gazebo::GazeboModelPluginDrag,
  gz::sim::System,
  astrobee_gazebo::GazeboModelPluginDrag::ISystemConfigure,
  astrobee_gazebo::GazeboModelPluginDrag::ISystemPreUpdate,
  astrobee_gazebo::GazeboModelPluginDrag::ISystemPostUpdate 
)

GZ_ADD_PLUGIN_ALIAS(astrobee_gazebo::GazeboModelPluginDrag, 
                    "astrobee_plugin_drag", 
                    "astrobee_gazebo::GazeboModelPluginDrag")
