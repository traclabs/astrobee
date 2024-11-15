/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <gz/sensors/WideAngleCameraSensor.hh>
#include <astrobee_gazebo/astrobee_gazebo.h>
#include <gz/sim/Util.hh>
// Transformation helper code
#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace astrobee_gazebo {

FF_DEFINE_LOGGER("gazebo");

// Constructor
FreeFlyerPlugin::FreeFlyerPlugin(std::string const& plugin_name,
  std::string const& plugin_frame, bool send_heartbeats) :
    ff_util::FreeFlyerComponent(plugin_name, send_heartbeats),
      robot_name_("/"), plugin_name_(plugin_name),
        plugin_frame_(plugin_frame), parent_frame_() {}

// Destructor
FreeFlyerPlugin::~FreeFlyerPlugin() {
  // nh_ff_.shutdown();
  this->executor_->cancel();
  this->thread_executor_spin_.join();
}

// Some plugins might want the world as the parent frame
void FreeFlyerPlugin::SetParentFrame(std::string const& parent) {
  parent_frame_ = parent;
}

// Code taken of Node::Get from gazebo_ros/src/node.cpp (classic)
// NOTE: We didn't copy the namespace part of the code. If you see errors, see the original code
std::shared_ptr<rclcpp::Node> FreeFlyerPlugin::GetNode(sdf::ElementPtr sdf, const std::string node_name)
{
  // Initialize arguments
  std::string name = "";
  std::string ns = "/";

  // Get the name of the plugin as the name for the node.
  if (!sdf->HasAttribute("name")) {
    FF_WARN("Name of plugin not found.");
  }

  if (!node_name.empty()) {
    name = node_name;
  } else {
    name = sdf->Get<std::string>("name");
  }

  return   rclcpp::Node::make_shared(name, ns);
}

// Load function
void FreeFlyerPlugin::InitializePlugin(std::string const& robot_name, std::string const& plugin_name,
                                       sdf::ElementPtr sdf) {
  gzwarn << "Starting plugin: " << plugin_name_ << ": "<<plugin_name << std::endl;
  robot_name_ = robot_name;

  // Ensure that ROS is setup
  if(!rclcpp::ok())
  {
     rclcpp::init(0, nullptr);  
  }
  // Get nodehandle based on the model.
  nh_ = GetNode(sdf, plugin_name);
  
  // Start a thread to spin the node
  this->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  this->executor_->add_node(nh_);
  auto spin = [this]() { this->executor_->spin(); };
  this->thread_executor_spin_ = std::thread(spin);
  
  
  // Initialize ROS node for Gazebo
  FreeFlyerComponent::FreeFlyerComponentGazeboInit(nh_, plugin_name);
  FF_DEBUG_STREAM("Loading " << plugin_name_  << " : "<< plugin_name << " on robot " << robot_name_);

  // Get nodehandle based on the model name.
  buffer_.reset(new tf2_ros::Buffer(nh_->get_clock()));
  listener_.reset(new tf2_ros::TransformListener(*buffer_));

  // Assign special node handles that use custom callback queues to avoid
  // Gazebo locking up heartbeats from being sent to the system monitor.
  // nh_ff_ = ros::NodeHandle(robot_name_);
  // Setup(nh_ff_, nh_ff_, plugin_name);

  // If we have a frame then defer chainloading until we receive them
  gzwarn << "Setting up extrinsics for " << plugin_name << std::endl;
  timer_.createTimer(5.0,
      std::bind(&FreeFlyerPlugin::SetupExtrinsics, this), nh_);
}

// Poll for extrinsics until found
void FreeFlyerPlugin::SetupExtrinsics() {
  gzwarn << "Setting up extrinsics... plugin frame: " << plugin_frame_.c_str() << std::endl;
  // If we don't need extrinsics, then don't bother looking...
  if (plugin_frame_.empty()) {
    if (ExtrinsicsCallback(nullptr))
      timer_.stop();
    return;
  }
  // Get the parent and child frame
  if (parent_frame_.empty())
    parent_frame_ = GetFrame(FRAME_NAME_BODY);
  // Keep trying to find the frame transform
  try { gzwarn << "Should be trying to get transfrom from " << parent_frame_ << " to: " << GetFrame() << std::endl;
    geometry_msgs::TransformStamped tf =
      buffer_->lookupTransform(parent_frame_, GetFrame(), ros::Time(0));
    if (ExtrinsicsCallback(&tf)) {
      OnExtrinsicsReceived(nh_);
      timer_.stop();
    }
  } catch (tf2::TransformException &ex) {}
}

// Get the extrinsics frame
std::string FreeFlyerPlugin::GetFrame(std::string target, std::string delim) {
  std::string frame = (target.empty() ? plugin_frame_ : target);
  return (robot_name_ == "/" ? frame : robot_name_ + delim + frame);
}

// Model plugin

// Constructor
FreeFlyerModelPlugin::FreeFlyerModelPlugin(std::string const& plugin_name,
  std::string const& plugin_frame, bool send_heartbeats) :
    FreeFlyerPlugin::FreeFlyerPlugin(
      plugin_name, plugin_frame, send_heartbeats) {

  link_ = nullptr;
  model_ = nullptr;
  world_entity_ = gz::sim::kNullEntity;
  update_extrinsics_ = false;
}

// Destructor
FreeFlyerModelPlugin::~FreeFlyerModelPlugin() {}

// Auto-called when Gazebo loads the plugin
void FreeFlyerModelPlugin::Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         gz::sim::EventManager &_eventMgr) {
  
  sdf_   = _sdf->Clone();

  model_entity_ = _entity;
  world_entity_ = gz::sim::kNullEntity;
  world_entity_ = gz::sim::worldEntity(_ecm);

  model_.reset( new gz::sim::Model(model_entity_) );
  link_entity_ = model_->CanonicalLink(_ecm);  
  link_.reset( new gz::sim::Link(link_entity_) );
  
  // Read namespace
  std::string ns = model_->Name(_ecm);
  if (ns == "bsharp")
    ns = "/";

  // Read plugin custom name if specified
  std::string plugin_name = "";
  if (sdf_->HasElement("plugin_name"))
    plugin_name = sdf_->Get<std::string>("plugin_name");
  // Read plugin custom frame if specified
  if (sdf_->HasElement("plugin_frame"))
    plugin_frame_ = sdf_->Get<std::string>("plugin_frame");

  // Initialize the FreeFlyerPlugin
  InitializePlugin(ns, plugin_name, sdf_);

  // Now load the rest of the plugin
  LoadCallback(nh_, _ecm);
}

void FreeFlyerModelPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm)
{

  if(update_extrinsics_)
  {
     model_->SetWorldPoseCmd(_ecm, extrinsics_pose_);
     update_extrinsics_ = false;
  }

  PreUpdate_(_info, _ecm);
}

// Get the model link
std::shared_ptr<gz::sim::Link> FreeFlyerModelPlugin::GetLink() {
  return link_;
}

// Get the model world
gz::sim::Entity FreeFlyerModelPlugin::GetWorld() {
  return world_entity_;
}

// Get the model
std::shared_ptr<gz::sim::Model> FreeFlyerModelPlugin::GetModel() {
  return model_;
}

// Get the model
//gz::sim::Entity FreeFlyerModelPlugin::GetModel() {
//  return model_entity_;
//}


// Manage the extrinsics based on the sensor type
bool FreeFlyerModelPlugin::ExtrinsicsCallback(
  geometry_msgs::TransformStamped const* tf) {
  // A tf nullptr means no transform is required
  if (tf) {
    // Handle the transform for all sensor types
    extrinsics_pose_ = gz::math::Pose3d(
      tf->transform.translation.x,
      tf->transform.translation.y,
      tf->transform.translation.z,
      tf->transform.rotation.w,
      tf->transform.rotation.x,
      tf->transform.rotation.y,
      tf->transform.rotation.z);
    // Set the model pose
    update_extrinsics_ = true;
  }
  // Success
  return true;
}

// Sensor plugin

// Constructor
FreeFlyerSensorPlugin::FreeFlyerSensorPlugin(std::string const& plugin_name,
  std::string const& plugin_frame, bool send_heartbeats) :
    FreeFlyerPlugin::FreeFlyerPlugin(
      plugin_name, plugin_frame, send_heartbeats) {
  update_extrinsics_  = false;      
}

// Destructor
FreeFlyerSensorPlugin::~FreeFlyerSensorPlugin() {}

// Sensor plugin load callback
void FreeFlyerSensorPlugin::Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         gz::sim::EventManager &_eventMgr) {

  sdf_ = _sdf->Clone();
  
  sensor_entity_ = _entity; 
  world_entity_ = gz::sim::worldEntity(sensor_entity_, _ecm);
  model_entity_ = gz::sim::topLevelModel(sensor_entity_, _ecm);
  
  model_.reset( new gz::sim::Model(model_entity_) );

  // Read namespace
  std::string ns = model_->Name(_ecm);
  if (ns == "bsharp")
    ns = "/";

  // Read plugin custom name if specified
  std::string plugin_name = "";
  if (sdf_->HasElement("plugin_name"))
    plugin_name = sdf_->Get<std::string>("plugin_name");
  // Read plugin custom frame if specified
  if (sdf_->HasElement("plugin_frame"))
    plugin_frame_ = sdf_->Get<std::string>("plugin_frame");

  // Initialize the FreeFlyerPlugin
  InitializePlugin(ns, plugin_name, sdf_);

  // Now load the rest of the plugin
  LoadCallback(nh_, _ecm);
}

// Get the sensor world
gz::sim::Entity FreeFlyerSensorPlugin::GetWorld() {
  return world_entity_;
}

// Get the sensor model
gz::sim::Entity FreeFlyerSensorPlugin::GetModel() {
  return model_entity_;
}

// Get sensor entity
gz::sim::Entity FreeFlyerSensorPlugin::GetSensor() {
  return sensor_entity_;
}

void FreeFlyerSensorPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                                      gz::sim::EntityComponentManager &_ecm)
{
  if(update_extrinsics_)
  {
     //model_->SetWorldPoseCmd(_ecm, extrinsics_pose_);
     update_extrinsics_ = false;
  }

  PreUpdate_(_info, _ecm);
}


// Manage the extrinsics
bool FreeFlyerSensorPlugin::ExtrinsicsCallback(
  geometry_msgs::TransformStamped const* tf) {
/*
  // A tf nullptr means no transform is required
  if (tf) {
    // Handle the transform for all sensor types
    gz::math::Pose3d pose(
      tf->transform.translation.x,
      tf->transform.translation.y,
      tf->transform.translation.z,
      tf->transform.rotation.w,
      tf->transform.rotation.x,
      tf->transform.rotation.y,
      tf->transform.rotation.z);

    // Set the sensor pose
    if (sensor_)
      sensor_->SetPose(pose);
    else
      return false;

    // Convert to a world pose
    Eigen::Quaterniond rot_90_x(0.70710678, 0.70710678, 0, 0);
    Eigen::Quaterniond rot_90_z(0.70710678, 0, 0, 0.70710678);
    Eigen::Quaterniond pose_temp(
      tf->transform.rotation.w,
      tf->transform.rotation.x,
      tf->transform.rotation.y,
      tf->transform.rotation.z);
    pose_temp = pose_temp * rot_90_x;
    pose_temp = pose_temp * rot_90_z;
    pose = gz::math::Pose3d(
      tf->transform.translation.x,
      tf->transform.translation.y,
      tf->transform.translation.z,
      pose_temp.w(), pose_temp.x(), pose_temp.y(), pose_temp.z());

    gz::math::Pose3d tf_bs = pose;
    gz::math::Pose3d tf_wb = model_->WorldPose();
    gz::math::Pose3d tf_ws = tf_bs + tf_wb;
    gz::math::Pose3d world_pose(tf_bs + tf_wb);

    // In the case of a camera update the camera world pose
    if (gz::sim::entityTypeId(sensor_entity_, ecm_) == gz::sim::components::Camera::typeId) {
      sensors::CameraSensorPtr sensor
        = std::dynamic_pointer_cast<sensors::CameraSensor>(sensor_);
      if (sensor && sensor->Camera())
        sensor->Camera()->SetWorldPose(world_pose);
      else
        return false;
    }

    // In the case of a wide angle camera update the camera world pose
    if (gz::sim::entityTypeId(sensor_entity_, ecm)  == gz::sim::components::WideAngleCamera::typeId) {
      std::shared_ptr<sensors::WideAngleCameraSensor> sensor = gz::sim::components::WideAngleCamera(sensor_entity_);
        //std::dynamic_pointer_cast<sensors::WideAngleCameraSensor>(sensor_);
      if (sensor && sensor->Camera())
        sensor->Camera()->SetWorldPose(world_pose);
      else
        return false;
    }

    // In the case of a depth camera update the depth camera pose
    if (gz::sim::entityTypeId(sensor_entity_, ecm)  == gz::sim::components::DepthCamera::typeId) {
      sensors::DepthCameraSensorPtr sensor =
        std::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensor_);
      if (sensor && sensor->DepthCamera())
        sensor->DepthCamera()->SetWorldPose(world_pose);
      else
        return false;
    }
  }*/
  // Success
  return true;
}

// Compute the transform from sensor to world coordinates
Eigen::Affine3d SensorToWorld(gz::math::Pose3d const& world_pose,
                              gz::math::Pose3d const& sensor_pose) {
    Eigen::Affine3d body_to_world
      = (Eigen::Translation3d(world_pose.Pos().X(),
                              world_pose.Pos().Y(),
                              world_pose.Pos().Z()) *
         Eigen::Quaterniond(world_pose.Rot().W(),
                            world_pose.Rot().X(),
                            world_pose.Rot().Y(),
                            world_pose.Rot().Z()));
    Eigen::Affine3d sensor_to_body
      = (Eigen::Translation3d(sensor_pose.Pos().X(),
                              sensor_pose.Pos().Y(),
                              sensor_pose.Pos().Z()) *
         Eigen::Quaterniond(sensor_pose.Rot().W(),
                            sensor_pose.Rot().X(),
                            sensor_pose.Rot().Y(),
                            sensor_pose.Rot().Z()));
    return body_to_world * sensor_to_body;
}
/*
void FillCameraInfo(rendering::CameraPtr camera, sensor_msgs::CameraInfo & msg) {
  msg.width = camera->ImageWidth();
  msg.height = camera->ImageHeight();

  double hfov = camera->HFOV().Radian();  // horizontal field of view in radians
  double focal_length = camera->ImageWidth()/(2.0 * tan(hfov/2.0));
  double opitcal_center_x = msg.width/2.0;
  double optical_center_y = msg.height/2.0;

  // Intrinsics matrix
  msg.k = {focal_length, 0, opitcal_center_x,
           0, focal_length, optical_center_y,
           0, 0, 1};

  // Projection matrix. We won't use this, but initalize it to something.
  msg.p = {1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 0};

  // Rotation matrix. We won't use it.
  msg.r = {1, 0, 0,
           0, 1, 0,
           0, 0, 1};

  rendering::DistortionPtr dPtr = camera->LensDistortion();

  // sensor_msgs::CameraInfo can manage only a few distortion
  // models. Here we assume plumb_bob just to pass along the
  // coefficients. Out of all the simulated cameras, nav_cam is
  // fisheye, which uses only K1, and all others have zero
  // distortion. Hence this code was not tested in the most general
  // setting.
  msg.distortion_model = "plumb_bob";
  if (dPtr) {
    #if GAZEBO_MAJOR_VERSION > 8
    msg.d = {camera->LensDistortion()->K1(),
             camera->LensDistortion()->K2(),
             camera->LensDistortion()->K3(),
             camera->LensDistortion()->P1(),
             camera->LensDistortion()->P2()};
    #else
    msg.D = {camera->LensDistortion()->GetK1(),
             camera->LensDistortion()->GetK2(),
             camera->LensDistortion()->GetK3(),
             camera->LensDistortion()->GetP1(),
             camera->LensDistortion()->GetP2()};
    #endif
  } else {
    msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  }
}*/

} // namespace astrobee_gazebo
