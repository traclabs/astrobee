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

// Tf2 includes
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

// Messages
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
namespace geometry_msgs {
typedef msg::TransformStamped TransformStamped;
typedef msg::PoseStamped PoseStamped;
typedef msg::TwistStamped TwistStamped;
}  // namespace geometry_msgs

// STL includes
#include <string>

namespace astrobee_gazebo {

// This class is a plugin that calls the GNC autocode to predict
// the force to be applied to the rigid body
class GazeboModelPluginTruth : public FreeFlyerModelPlugin {
 public:
  GazeboModelPluginTruth() :
    FreeFlyerModelPlugin("gazebo_truth", ""), rate_(62.5), tf_(true),
      pose_(true), twist_(true), static_(false),
        parent_("world"), child_("truth") {}

  ~GazeboModelPluginTruth() {}

 protected:
  // Called when the plugin is loaded into the simulator
  void LoadCallback(NodeHandle &nh, gz::sim::EntityComponentManager &_ecm) {
    // If we specify a frame name different to our sensor tag name
    if (sdf_->HasElement("rate"))
      rate_ = sdf_->Get<double>("rate");
    if (sdf_->HasElement("parent"))
      parent_ = sdf_->Get<std::string>("parent");
    if (sdf_->HasElement("child"))
      child_ = sdf_->Get<std::string>("child");
    if (sdf_->HasElement("pose"))
      pose_ = sdf_->Get<bool>("pose");
    if (sdf_->HasElement("twist"))
      twist_ = sdf_->Get<bool>("twist");
    if (sdf_->HasElement("tf"))
      tf_ = sdf_->Get<bool>("tf");
    if (sdf_->HasElement("static"))
      static_ = sdf_->Get<bool>("static");

    // Setup TF2 message
    msg_.header.frame_id = parent_;
    msg_.child_frame_id = GetFrame(child_);

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*nh);

    // Enable velocity checks
    // (by default it seems Gazebo does not publish links' velocities,
    // so we have to explicitly request them to be published if using twist=true
    GetLink()->EnableVelocityChecks(_ecm, true);

    // If we are
    if (static_) {
      msg_.header.stamp = GetTimeNow();
      gz::math::Pose3d pose = gz::sim::worldPose(model_->Entity(), _ecm);
      msg_.transform.translation.x = pose.Pos().X();
      msg_.transform.translation.y = pose.Pos().Y();
      msg_.transform.translation.z = pose.Pos().Z();
      msg_.transform.rotation.x = pose.Rot().X();
      msg_.transform.rotation.y = pose.Rot().Y();
      msg_.transform.rotation.z = pose.Rot().Z();
      msg_.transform.rotation.w = pose.Rot().W();
      tf_broadcaster_->sendTransform(msg_);
      return;
    }

    // Ground truth
    pub_truth_pose_ = FF_CREATE_PUBLISHER(nh, geometry_msgs::PoseStamped, TOPIC_LOCALIZATION_TRUTH, 1);
    pub_truth_twist_ = FF_CREATE_PUBLISHER(nh, geometry_msgs::TwistStamped, TOPIC_LOCALIZATION_TRUTH_TWIST, 1);

    // Called before each iteration of simulated world update
    timer_.createTimer(1 / rate_,
      std::bind(&GazeboModelPluginTruth::TimerCallback, this), nh, false, true);
  }

  // Called on simulation reset
  void Reset() {}

  // Called on every discrete time tick in the simulated world
  void TimerCallback() {
    msg_.header.stamp = GetTimeNow();
    // If the rate is higher than the sim time, prevent repeated timestamps
    bool publish_tf = true;
    if (msg_.header.stamp == last_time_) {
      publish_tf = false;
    } else {
      last_time_ = msg_.header.stamp;
    }

    std::lock_guard<std::mutex> guard(mutex_data_);

    if(!model_pose_ || !model_linear_vel_ || !model_angular_vel_)
      return;    

    if (tf_ && publish_tf) {
      msg_.transform.translation.x = model_pose_.value().Pos().X();
      msg_.transform.translation.y = model_pose_.value().Pos().Y();
      msg_.transform.translation.z = model_pose_.value().Pos().Z();
      msg_.transform.rotation.x = model_pose_.value().Rot().X();
      msg_.transform.rotation.y = model_pose_.value().Rot().Y();
      msg_.transform.rotation.z = model_pose_.value().Rot().Z();
      msg_.transform.rotation.w = model_pose_.value().Rot().W();
      tf_broadcaster_->sendTransform(msg_);
    }
    // Pose
    if (pose_) {
      ros_truth_pose_.header = msg_.header;
      ros_truth_pose_.pose.position.x = model_pose_.value().Pos().X();
      ros_truth_pose_.pose.position.y = model_pose_.value().Pos().Y();
      ros_truth_pose_.pose.position.z = model_pose_.value().Pos().Z();
      ros_truth_pose_.pose.orientation.x = model_pose_.value().Rot().X();
      ros_truth_pose_.pose.orientation.y = model_pose_.value().Rot().Y();
      ros_truth_pose_.pose.orientation.z = model_pose_.value().Rot().Z();
      ros_truth_pose_.pose.orientation.w = model_pose_.value().Rot().W();
      pub_truth_pose_->publish(ros_truth_pose_);
    }
    // Twist
    if (twist_) {
      ros_truth_twist_.header = msg_.header;
      ros_truth_twist_.twist.linear.x = model_linear_vel_.value().X();
      ros_truth_twist_.twist.linear.y = model_linear_vel_.value().Y();
      ros_truth_twist_.twist.linear.z = model_linear_vel_.value().Z();
      ros_truth_twist_.twist.angular.x = model_angular_vel_.value().X();
      ros_truth_twist_.twist.angular.y = model_angular_vel_.value().Y();
      ros_truth_twist_.twist.angular.z = model_angular_vel_.value().Z();
      pub_truth_twist_->publish(ros_truth_twist_);
    }
    
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) 
  {
  }

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) 
  {
     std::lock_guard<std::mutex> guard(mutex_data_);
     model_pose_ = GetLink()->WorldPose(_ecm);  
     model_linear_vel_ = GetLink()->WorldLinearVelocity(_ecm);
     model_angular_vel_ = GetLink()->WorldAngularVelocity(_ecm);
  }                


 private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double rate_;
  bool tf_, pose_, twist_, static_;
  std::string parent_, child_;
  
  // Storing data
  std::optional<gz::math::Vector3d> model_linear_vel_;
  std::optional<gz::math::Vector3d> model_angular_vel_;
  std::optional<gz::math::Pose3d> model_pose_;
  std::mutex mutex_data_;
  
  // Publishing to ROS2 side
  geometry_msgs::TransformStamped msg_;
  geometry_msgs::PoseStamped ros_truth_pose_;
  geometry_msgs::TwistStamped ros_truth_twist_;
  
  rclcpp::Publisher<geometry_msgs::PoseStamped>::SharedPtr pub_truth_pose_;
  rclcpp::Publisher<geometry_msgs::TwistStamped>::SharedPtr pub_truth_twist_;
  ff_util::FreeFlyerTimer timer_;
  rclcpp::Time last_time_;
};

}   // namespace astrobee_gazebo

// Register this plugin with the simulator
GZ_ADD_PLUGIN(
  astrobee_gazebo::GazeboModelPluginTruth,
  gz::sim::System,
  astrobee_gazebo::GazeboModelPluginTruth::ISystemConfigure,
  astrobee_gazebo::GazeboModelPluginTruth::ISystemPreUpdate,
  astrobee_gazebo::GazeboModelPluginTruth::ISystemPostUpdate 
)
