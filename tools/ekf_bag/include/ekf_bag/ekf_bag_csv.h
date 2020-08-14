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

#ifndef EKF_BAG_EKF_BAG_CSV_H_
#define EKF_BAG_EKF_BAG_CSV_H_

#include <ekf_bag/ekf_bag.h>
#include <ekf_bag/tracked_features.h>

#include <string>

namespace ekf_bag {

class EkfBagCsv : public EkfBag {
 public:
  EkfBagCsv(const char* bagfile, const char* mapfile, const char* csvfile,
      bool run_ekf = true, bool gen_features = true, const char* biasfile = NULL,
      std::string image_topic = std::string(TOPIC_HARDWARE_NAV_CAM), const std::string& gnc_config = "gnc.config");
  virtual ~EkfBagCsv(void);

 protected:
  virtual void UpdateGroundTruth(const geometry_msgs::PoseStamped & pose);

  virtual void UpdateEKF(const ff_msgs::EkfState & state);
  virtual void UpdateOpticalFlow(const ff_msgs::Feature2dArray & of);
  virtual void UpdateSparseMap(const ff_msgs::VisualLandmarks & vl);
  virtual void UpdateOpticalFlowReg(const ff_msgs::CameraRegistration & reg);
  virtual void UpdateSparseMapReg(const ff_msgs::CameraRegistration & reg);

  virtual void ReadParams(config_reader::ConfigReader* config);

  TrackedOFFeatures tracked_of_;

 private:
  FILE* f_;

  bool start_time_set_;
  ros::Time start_time_;
  ros::Time ml_reg_time_, of_reg_time_;
  unsigned int ml_cam_id_, of_cam_id_;
};

}  // end namespace ekf_bag

#endif  // EKF_BAG_EKF_BAG_CSV_H_

