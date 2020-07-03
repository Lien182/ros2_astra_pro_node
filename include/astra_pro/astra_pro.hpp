// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#ifndef ASTRA_PRO__ASTRA_PRO_HPP_
#define ASTRA_PRO__ASTRA_PRO_HPP_

#include <rclcpp/rclcpp.hpp>
#include <astra_pro/visibility.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <string>

namespace astra_pro
{

class AstraPro : public rclcpp::Node
{
public:
  explicit AstraPro(const rclcpp::NodeOptions & options);

protected:
  void onInit();
  void doWork();
  void reconfigureCallback();

private:
  image_transport::CameraPublisher pub_;

  cv::VideoCapture cap_;
  cv::Mat image_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string filename_;
  bool flip_horizontal_;
  bool flip_vertical_;

  std::string frame_id_;
  double publish_rate_;
  std::string camera_info_url_;
  bool flip_image_;
  int flip_value_;
  sensor_msgs::msg::CameraInfo camera_info_;
};

}  // namespace image_publisher

#endif  // IMAGE_PUBLISHER__IMAGE_PUBLISHER_HPP_