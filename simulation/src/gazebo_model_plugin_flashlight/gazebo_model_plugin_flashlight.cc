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
#include <gz/msgs/entity_factory.pb.h>
#include <gz/plugin/Register.hh>

// Transformation helper code
#include <tf2_ros/transform_listener.h>

// RVIZ visualization
#include <visualization_msgs/msg/marker_array.hpp>
namespace visualization_msgs {
typedef msg::Marker Marker;
typedef msg::MarkerArray MarkerArray;
}  // namespace visualization_msgs

// Freeflyer messages
#include <ff_hw_msgs/srv/set_flashlight.hpp>
namespace ff_hw_msgs {
typedef srv::SetFlashlight SetFlashlight;
}  // namespace ff_hw_msgs

// STL includes
#include <string>
#include <map>

namespace astrobee_gazebo {

FF_DEFINE_LOGGER("gazebo_model_plugin_perching_flashlight");

class GazeboModelPluginFlashlight : public FreeFlyerModelPlugin {
 public:
  GazeboModelPluginFlashlight() : FreeFlyerModelPlugin("",
    "", true), rate_(10.0),
      width_(0.03), height_(0.02), depth_(0.005) {}

  ~GazeboModelPluginFlashlight() {

  }

 protected:
  // Called when the plugin is loaded into the simulator
  void LoadCallback(NodeHandle &nh, gz::sim::EntityComponentManager &_ecm) {
    // Get parameters from the SDF
    if (sdf_->HasElement("rate"))
      rate_ = sdf_->Get<double>("rate");
    if (sdf_->HasElement("width"))
      width_ = sdf_->Get<double>("width");
    if (sdf_->HasElement("height"))
      height_ = sdf_->Get<double>("height");
    if (sdf_->HasElement("depth"))
      depth_ = sdf_->Get<double>("depth");
    if (sdf_->HasElement("plugin_frame"))
      plugin_frame_ = sdf_->Get<std::string>("plugin_frame");

    // Service names
    std::string world_name = "default";
    
    std::optional<std::string> world_name_opt = gz::sim::World(world_entity_).Name(_ecm);
    if( world_name_opt )
      world_name = world_name_opt.value();

    create_entity_srv_ = "/world/" + world_name + "/create";
    modify_light_srv_ = "/world/" + world_name + "/light_config";

    // Use the message system to toggle visibility of visual elements
  
    // For the RVIZ marker array
    pub_rviz_ = FF_CREATE_PUBLISHER(nh_, visualization_msgs::MarkerArray, TOPIC_HARDWARE_LIGHTS_RVIZ, 0);

    // Rotate from the flaslight frame to the visual frame
    pose_ = gz::math::Pose3d(0.0, 0, 0, 0.70710678, 0, -0.70710678, 0);


    // Create the visual
    /*gz::sim::SdfEntityCreator sec(_ecm, _em);
    sdf::Visual* visual = new sdf::Visual();
    visual->SetName(GetFrame(plugin_frame_ + "_visual", "_"));
    //visual->set_parent_name(GetModel()->GetLink()->GetScopedName());
    sdf::Box box;
    box.SetSize(gz::math::Vector3d(depth_, width_, height_));
    sdf::Geom geom; 
    geom.setBoxShape(box); // do we need to say setTypeAs well?
    visual->SetGeom(geom);
    visual->SetType(sdf::GeometryType::BOX);
    visual->SetCastShadows(false);
    visual->SetTransparency(1.0);
    visual->setRawPose(pose_);
    std::Material material;
    material.SetScriptName("Astrobee/Flashlight");
    visual->setMaterial(material);
    visual->setVisibilityFlags(1); // when non-zero, visible to the camera
    //visual_.set_is_static(false);
    //visual_.set_visible(true);

    sec.CreateEntities(visual);*/

  }

  // Only send measurements when extrinsics are available
  void OnExtrinsicsReceived(NodeHandle &nh) {
    srv_ = nh->create_service<ff_hw_msgs::SetFlashlight>("hw/" + plugin_frame_ + "/control",
      std::bind(&GazeboModelPluginFlashlight::ToggleCallback, this, std::placeholders::_1, std::placeholders::_2));
  }


  // Manage the extrinsics based on the sensor type
  bool ExtrinsicsCallback(geometry_msgs::TransformStamped const* tf) {
    if (!tf) {
      FF_WARN("Flashlight extrinsics are null");
      return false;
    }
    FF_WARN("EXTRINSICS CALLBACK............");
    // Create the rviz marker
    marker_.header.stamp = GetTimeNow();
    marker_.header.frame_id = GetFrame();
    marker_.ns = GetFrame(plugin_frame_, "_");
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::CUBE;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose.position.x = pose_.Pos().X();
    marker_.pose.position.y = pose_.Pos().Y();
    marker_.pose.position.z = pose_.Pos().Z();
    marker_.pose.orientation.x = pose_.Rot().X();
    marker_.pose.orientation.y = pose_.Rot().Y();
    marker_.pose.orientation.z = pose_.Rot().Z();
    marker_.pose.orientation.w = pose_.Rot().W();
    marker_.scale.x = depth_;
    marker_.scale.y = width_;
    marker_.scale.z = height_;
    marker_.color.a = 0.0;
    marker_.color.r = 1.0;
    marker_.color.g = 1.0;
    marker_.color.b = 1.0;
    visualization_msgs::MarkerArray msg;
    msg.markers.push_back(marker_);
    pub_rviz_->publish(msg);

    // Aggregate pose
    pose_ = pose_ + gz::math::Pose3d(
      tf->transform.translation.x,
      tf->transform.translation.y,
      tf->transform.translation.z,
      tf->transform.rotation.w,
      tf->transform.rotation.x,
      tf->transform.rotation.y,
      tf->transform.rotation.z);

    // Create the Gazebo visual
    // Need to use this pose!! Update
    //pub_visual_->Publish(visual_);

    // Create the gazebo light    
    light_req_.mutable_light()->set_name(GetFrame(plugin_frame_ + "_front_light", "_"));
    light_req_.mutable_light()->set_type(gz::msgs::Light::SPOT);
    light_req_.mutable_light()->set_attenuation_constant(1.0);
    light_req_.mutable_light()->set_attenuation_linear(0.02);
    light_req_.mutable_light()->set_attenuation_quadratic(0.0);
    light_req_.mutable_light()->set_range(10);
    light_req_.mutable_light()->set_cast_shadows(false);
    light_req_.mutable_light()->set_spot_inner_angle(0.6);
    light_req_.mutable_light()->set_spot_outer_angle(2.2);
    light_req_.mutable_light()->set_spot_falloff(1.0);

    // For some reason common::Color stopped existing in later versions
    gz::msgs::Set(light_req_.mutable_light()->mutable_diffuse(), gz::math::Color(0.5, 0.5, 0.5, 1));
    gz::msgs::Set(light_req_.mutable_light()->mutable_specular(), gz::math::Color(0.1, 0.1, 0.1, 1));
  
    if(world_pose_)
    {
      mutex_data_.lock();
      gz::msgs::Set(light_req_.mutable_light()->mutable_pose(), 
                pose_ + world_pose_.value());
      mutex_data_.unlock();
    }
    bool res = sendEntityRequest(create_entity_srv_, light_req_);

    // Success
    return true;
  }

  bool sendEntityRequest(const std::string &_srv_name, 
                         const gz::msgs::EntityFactory &_req, 
                         const unsigned int &_timeout = 500)
  {
    gz::msgs::Boolean res;
    bool result;
  
    if( gz_node_.Request(_srv_name, _req, _timeout, res, result) )
    {
      if(result)
      {
        FF_WARN("YES, SUCCESS IN CALLING SERVICE: %s !!!!!!!!!!!!!!1", _srv_name.c_str());
        return true;
      }
      else
        FF_ERROR("Error ws in result error back");
    } else
    {
      FF_ERROR("ERROR WAS IN CALLING SERVICE: %s", _srv_name.c_str());
    }

    FF_ERROR("Failed either creating the service or getting false return for entityRequest");
    return false;
  }

  // Called when the laser needs to be toggled
  bool ToggleCallback(const std::shared_ptr<ff_hw_msgs::SetFlashlight::Request> req,
                      std::shared_ptr<ff_hw_msgs::SetFlashlight::Response> res) {
    // Update the alpha channel in rviz
    marker_.header.stamp = GetTimeNow();
    marker_.color.a = static_cast<double>(req->brightness) / 200.0;
    visualization_msgs::MarkerArray msg;
    msg.markers.push_back(marker_);
    pub_rviz_->publish(msg);

    // Update Gazebo visual
    //visual_.set_transparency(1.0 - marker_.color.a);
    //pub_visual_->Publish(visual_);

    // Update the gazebo light
    light_req_.mutable_light()->set_attenuation_constant(1.0 - marker_.color.a);
    sendEntityRequest(modify_light_srv_, light_req_);

    // Print response
    res->success = true;
    res->status_message = "Flashlight toggled successfully";
    return true;
  }


  virtual void PreUpdate_(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override {
    WorldUpdateBegin();
  }

  // Modify the new entity to be only visible in the GUI
  // Called when a new entity is created
  void WorldUpdateBegin() {
    if(!world_pose_)
      return;
    
    mutex_data_.lock();
    gz::math::Pose3d world_pose_now = world_pose_.value();
    mutex_data_.unlock();

    gz::msgs::Set(light_req_.mutable_light()->mutable_pose(), pose_ +
       world_pose_now);

    sendEntityRequest(modify_light_srv_, light_req_);
  }

  virtual void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override {

    mutex_data_.lock();
    world_pose_ = gz::sim::worldPose(model_entity_, _ecm);
    mutex_data_.unlock();
  }

 private:
  double rate_, width_, height_, depth_;
  gz::transport::Node gz_node_; 
  ff_util::FreeFlyerTimer timer_;
  rclcpp::Service<ff_hw_msgs::SetFlashlight>::SharedPtr srv_;
  rclcpp::Publisher<visualization_msgs::MarkerArray>::SharedPtr pub_rviz_;
  visualization_msgs::Marker marker_;
  gz::msgs::EntityFactory light_req_;
  gz::math::Pose3d pose_;
  std::optional<gz::math::Pose3d> world_pose_;
  std::mutex mutex_data_;
  std::string plugin_frame_ = "";
  std::string create_entity_srv_;
  std::string modify_light_srv_;
};

}   // namespace astrobee_gazebo

// Register this plugin with the simulator
GZ_ADD_PLUGIN(
  astrobee_gazebo::GazeboModelPluginFlashlight,
  gz::sim::System,
  astrobee_gazebo::GazeboModelPluginFlashlight::ISystemConfigure,
  astrobee_gazebo::GazeboModelPluginFlashlight::ISystemPreUpdate,
  astrobee_gazebo::GazeboModelPluginFlashlight::ISystemPostUpdate 
)

GZ_ADD_PLUGIN_ALIAS(astrobee_gazebo::GazeboModelPluginFlashlight, 
                    "astrobee_plugin_flashlight", 
                    "astrobee_gazebo::GazeboModelPluginFlashlight")

