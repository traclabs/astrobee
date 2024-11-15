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

// FSW includes
#include <config_reader/config_reader.h>

// Sensor plugin interface
#include <astrobee_gazebo/astrobee_gazebo.h>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Camera.hh>
#include <gz/sim/components/WideAngleCamera.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/plugin/Register.hh>

// Messages
#include <geometry_msgs/msg/pose_stamped.hpp>
namespace geometry_msgs {
typedef msg::PoseStamped PoseStamped;
}  // namespace geometry_msgs

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
namespace sensor_msgs {
typedef msg::Image Image;
typedef msg::CameraInfo CameraInfo;
}  // namespace sensor_msgs

// STL includes
#include <string>

namespace astrobee_gazebo {
FF_DEFINE_LOGGER("gazebo_sensor_plugin_nav_cam");
class GazeboSensorPluginNavCam : public FreeFlyerSensorPlugin {
 public:
  GazeboSensorPluginNavCam() : FreeFlyerSensorPlugin("nav_cam", "nav_cam", true), rate_(0.0), current_active_rate_(0.0) {}
	
  ~GazeboSensorPluginNavCam() {
  }

 protected:
  // Called when plugin is loaded into gazebo
  void LoadCallback(NodeHandle &nh, gz::sim::EntityComponentManager &_ecm) { 

    auto sensor_comp = _ecm.Component<gz::sim::components::WideAngleCamera>(GetSensor());
    if(!sensor_comp)
    {
        gzerr << "Plugin needs to be inside a WideAngleCamera sensor! \n";
        return;    
    }
    std::optional<std::string> topic = sensor_comp->Data().Topic();
    if(!topic)
    {
        gzerr << "Need to define topic for WideAngleCamera!";
        return;
    }
    
    sensor_topic_ = topic.value();

    // Set image constants
    image_msg_.is_bigendian = false;
    image_msg_.header.frame_id = GetFrame();
    image_msg_.encoding = sensor_msgs::image_encodings::MONO8;

    // Create a publisher
    pub_img_ = FF_CREATE_PUBLISHER(nh, sensor_msgs::Image, TOPIC_HARDWARE_NAV_CAM, 1);

    pub_pose_ = FF_CREATE_PUBLISHER(nh, geometry_msgs::PoseStamped, TOPIC_NAV_CAM_SIM_POSE, 10);
    pub_info_ = FF_CREATE_PUBLISHER(nh, sensor_msgs::CameraInfo, TOPIC_NAV_CAM_SIM_INFO, 10);

    // Read configuration
    config_reader::ConfigReader config;
    config.AddFile("simulation/simulation.config");
    if (!config.ReadFiles()) {
      FF_FATAL("Failed to read simulation config file.");
      return;
    }
    bool dos = true;
    if (!config.GetBool("disable_cameras_on_speedup", &dos))
      FF_FATAL("Could not read the disable_cameras_on_speedup parameter.");
    if (!config.GetReal("nav_cam_rate", &rate_)) FF_FATAL("Could not read the nav_cam_rate parameter.");
    config.Close();

    // If we have a sped up simulation and we need to disable the camera
    double simulation_speed = 1.0;
    if (nh->get_parameter("/simulation_speed", simulation_speed))
      if (simulation_speed > 1.0 && dos) rate_ = 0.0;

    // Toggle if the camera is active or not
    timer_toggle_.createTimer(0.5,
      std::bind(&GazeboSensorPluginNavCam::ToggleCallback, this), nh, false, true);
  }

  // Only send measurements when extrinsics are available
  void OnExtrinsicsReceived(NodeHandle& nh) {
    // Connect to the camera topic.
    gz_node_.Subscribe(sensor_topic_, &GazeboSensorPluginNavCam::ImageCallback, this);
  }

  // Turn camera on or off based on topic subscription
  void ToggleCallback() {
    
    double new_rate;
    if (pub_img_->get_subscription_count() > 0 && rate_ > 0) {      
      new_rate = rate_;
      gz_node_.Subscribe(sensor_topic_, &GazeboSensorPluginNavCam::ImageCallback, this);
    } else {
      new_rate = 0.0001;  
      gz_node_.Unsubscribe(sensor_topic_);
    }
    
    // Update rate if needed
    if( current_active_rate_ != new_rate )
    {
        setRate(new_rate);
        current_active_rate_ = new_rate;
    }

  }

  void setRate(double _new_rate)
  {
     gz::msgs::Double req;
     req.set_data(_new_rate);
     gz_node_.Request(sensor_topic_ + "/set_rate", req);
  }

  // Called when a new image must be rendered
  void ImageCallback(const gz::msgs::Image &_msg) {

    // Check that camera is mono
    // ANA MIGRATION HACK -- WHILE THEY FIX THE WIDE ANGLE CAMERA TO BE ABLE TO RUN WITH TYPE L8
    //if ( _msg.pixel_format_type() != gz::msgs::L_INT8 ) 
    //	FF_FATAL_STREAM("Camera format must be L_INT8");
    int num_channels = 1;
    int octets_per_channel = 1;
    if( _msg.pixel_format_type() == gz::msgs::L_INT8 )
    {
       image_msg_.encoding = sensor_msgs::image_encodings::MONO8;
       num_channels = 1;
       octets_per_channel = 1;
    }
    else if( _msg.pixel_format_type() == gz::msgs::RGB_INT8)
    {
       image_msg_.encoding = sensor_msgs::image_encodings::RGB8;
       num_channels = 3;
       octets_per_channel = 1;
    }
    // Quickly record the current time and current pose before doing other computations
    rclcpp::Time curr_time = GetTimeNow();
    // Publish the nav cam pose
    /*Eigen::Affine3d sensor_to_world = SensorToWorld(GetModel()->WorldPose(), sensor_->Pose());
    pose_msg_.header.frame_id = GetFrame();
    pose_msg_.header.stamp = curr_time;  // it is very important to get the time right
    pose_msg_.pose.position.x = sensor_to_world.translation().x();
    pose_msg_.pose.position.y = sensor_to_world.translation().y();
    pose_msg_.pose.position.z = sensor_to_world.translation().z();
    Eigen::Quaterniond q(sensor_to_world.rotation());
    pose_msg_.pose.orientation.w = q.w();
    pose_msg_.pose.orientation.x = q.x();
    pose_msg_.pose.orientation.y = q.y();
    pose_msg_.pose.orientation.z = q.z();
    pub_pose_->publish(pose_msg_);*/

    // Publish the nav cam intrinsics
    info_msg_.header.frame_id = GetFrame();
    info_msg_.header.stamp = curr_time;            // it is very important to get the time right
    //FillCameraInfo(sensor_->Camera(), info_msg_);  // fill in from the camera pointer
    //pub_info_->publish(info_msg_);

    // Publish the nav cam image  
    image_msg_.header.stamp.sec = _msg.header().stamp().sec();
    image_msg_.header.stamp.nanosec = _msg.header().stamp().nsec();
    image_msg_.height = _msg.height();
    image_msg_.width = _msg.width();
    image_msg_.step = _msg.step(); //image_msg_.width; // width * channel * octate_per_channel
    image_msg_.data.resize(image_msg_.step * image_msg_.height);
    memcpy(image_msg_.data.data(), _msg.data().c_str(), _msg.data().size());
    pub_img_->publish(image_msg_);
  }

 private:
  ff_util::FreeFlyerTimer timer_toggle_;
  sensor_msgs::Image image_msg_;
  geometry_msgs::PoseStamped pose_msg_;
  sensor_msgs::CameraInfo info_msg_;
  rclcpp::Publisher<sensor_msgs::Image>::SharedPtr pub_img_;
  rclcpp::Publisher<geometry_msgs::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<sensor_msgs::CameraInfo>::SharedPtr pub_info_;
  double rate_;
  double current_active_rate_;
};

}  // namespace astrobee_gazebo

// Register this plugin with the simulator
GZ_ADD_PLUGIN(
  astrobee_gazebo::GazeboSensorPluginNavCam,
  gz::sim::System,
  astrobee_gazebo::GazeboSensorPluginNavCam::ISystemConfigure,
  astrobee_gazebo::GazeboSensorPluginNavCam::ISystemPreUpdate,
  astrobee_gazebo::GazeboSensorPluginNavCam::ISystemPostUpdate 
)

GZ_ADD_PLUGIN_ALIAS(astrobee_gazebo::GazeboSensorPluginNavCam, 
                    "astrobee_plugin_nav_cam", 
                    "astrobee_gazebo::GazeboSensorPluginNavCam")