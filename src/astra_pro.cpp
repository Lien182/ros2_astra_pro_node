// Copyright (c) 2014, JSK Lab.
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

#include <camera_info_manager/camera_info_manager.h>

#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "astra_pro/astra_pro.hpp"

#include <astra/astra.hpp>
#include <cstdio>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <key_handler.h>

namespace astra_pro
{


class FrameListener : public astra::FrameListener
{
private:
    using depthbuffer_ptr = std::unique_ptr<int16_t []>;
    depthbuffer_ptr depthbuffer_;
    unsigned int depthlastWidth_;
    unsigned int depthlastHeight_;

    using colorbuffer_ptr = std::unique_ptr<astra::RgbPixel []>;
    colorbuffer_ptr colorbuffer_;
    unsigned int colorlastWidth_;
    unsigned int colorlastHeight_;

public:
    virtual void on_frame_ready(astra::StreamReader& reader,
                                astra::Frame& frame) override
    {
        const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();

        if (depthFrame.is_valid())
        {
            auto depthStream = reader.stream<astra::DepthStream>();
            print_depth(depthFrame, depthStream.coordinateMapper());
            check_fps();
        }

        const astra::ColorFrame colorFrame = frame.get<astra::ColorFrame>();

        if (colorFrame.is_valid())
        {
            print_color(colorFrame);
        }

    }

    void print_depth(const astra::DepthFrame& depthFrame,
                     const astra::CoordinateMapper& mapper)
    {
        if (depthFrame.is_valid())
        {
            int width = depthFrame.width();
            int height = depthFrame.height();
            int frameIndex = depthFrame.frame_index();

            //determine if buffer needs to be reallocated
            if (width != depthlastWidth_ || height != depthlastHeight_)
            {
                depthbuffer_ = depthbuffer_ptr(new int16_t[depthFrame.length()]);
                depthlastWidth_ = width;
                depthlastHeight_ = height;
            }
            depthFrame.copy_to(depthbuffer_.get());

            size_t index = ((width * (height / 2.0f)) + (width / 2.0f));
            short middle = depthbuffer_[index];

            float worldX, worldY, worldZ;
            float depthX, depthY, depthZ;
            mapper.convert_depth_to_world(width / 2.0f, height / 2.0f, middle, worldX, worldY, worldZ);
            mapper.convert_world_to_depth(worldX, worldY, worldZ, depthX, depthY, depthZ);

            std::cout << "depth frameIndex: " << frameIndex
                      << " value: " << middle
                      << " wX: " << worldX
                      << " wY: " << worldY
                      << " wZ: " << worldZ
                      << " dX: " << depthX
                      << " dY: " << depthY
                      << " dZ: " << depthZ
                      << std::endl;
        }
    }

    void print_color(const astra::ColorFrame& colorFrame)
    {
        if (colorFrame.is_valid())
        {
            int width = colorFrame.width();
            int height = colorFrame.height();
            int frameIndex = colorFrame.frame_index();

            if (width != colorlastWidth_ || height != colorlastHeight_){
                colorbuffer_ = colorbuffer_ptr(new astra::RgbPixel[colorFrame.length()]);
                colorlastWidth_ = width;
                colorlastHeight_ = height;
            }
            colorFrame.copy_to(colorbuffer_.get());

            size_t index = ((width * (height / 2.0f)) + (width / 2.0f));
            astra::RgbPixel middle = colorbuffer_[index];

            std::cout << "color frameIndex: " << frameIndex
                      << " r: " << static_cast<int>(middle.r)
                      << " g: " << static_cast<int>(middle.g)
                      << " b: " << static_cast<int>(middle.b)
                      << std::endl;
        }
    }

    void check_fps()
    {
        const float frameWeight = 0.5;

        auto newTimepoint = clock_type::now();
        float frameDuration = std::chrono::duration_cast<duration_type>(newTimepoint - lastTimepoint_).count();

        frameDuration_ = frameDuration * frameWeight + frameDuration_ * (1 - frameWeight);
        lastTimepoint_ = newTimepoint;

        double fps = 1.0 / frameDuration_;

        auto precision = std::cout.precision();
        std::cout << std::fixed
                  << std::setprecision(1)
                  << fps << " fps ("
                  << std::setprecision(2)
                  << frameDuration_ * 1000.0 << " ms)"
                  << std::setprecision(precision)
                  << std::endl;
    }

private:
    using duration_type = std::chrono::duration<float>;
    float frameDuration_{ 0.0 };

    using clock_type = std::chrono::system_clock;
    std::chrono::time_point<clock_type> lastTimepoint_{clock_type::now()};
};




AstraPro::AstraPro(const rclcpp::NodeOptions & options)
: Node("AstraPro", options)
{

  std::cout << "test" << std::endl;
  pub_ = image_transport::create_camera_publisher(this, "image_raw");

  flip_horizontal_ = this->declare_parameter("flip_horizontal", false);
  flip_vertical_ = this->declare_parameter("flip_vertical", false);
  frame_id_ = this->declare_parameter("frame_id", std::string("camera"));
  publish_rate_ = this->declare_parameter("publish_rate", static_cast<double>(10));
  camera_info_url_ = this->declare_parameter("camera_info_url", std::string(""));

  auto param_change_callback =
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
    {
      RCLCPP_INFO(get_logger(), "param_change_callback");

      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto parameter : parameters) {
        if (parameter.get_name() == "filename") {
          filename_ = parameter.as_string();
          RCLCPP_INFO(get_logger(), "Reset filename as '%s'", filename_.c_str());
          AstraPro::onInit();
          return result;
        } else if (parameter.get_name() == "flip_horizontal") {
          flip_horizontal_ = parameter.as_bool();
          RCLCPP_INFO(get_logger(), "Reset flip_horizontal as '%d'", flip_horizontal_);
          AstraPro::onInit();
          return result;
        } else if (parameter.get_name() == "flip_vertical") {
          flip_vertical_ = parameter.as_bool();
          RCLCPP_INFO(get_logger(), "Reset flip_vertical as '%d'", flip_vertical_);
          AstraPro::onInit();
          return result;
        } else if (parameter.get_name() == "frame_id") {
          frame_id_ = parameter.as_string();
          RCLCPP_INFO(get_logger(), "Reset frame_id as '%s'", frame_id_.c_str());
        } else if (parameter.get_name() == "publish_rate") {
          publish_rate_ = parameter.as_double();
          RCLCPP_INFO(get_logger(), "Reset publish_rate as '%lf'", publish_rate_);
        } else if (parameter.get_name() == "camera_info_url") {
          camera_info_url_ = parameter.as_string();
          RCLCPP_INFO(get_logger(), "Reset camera_info_rul as '%s'", camera_info_url_.c_str());
        } else
        {
          AstraPro::onInit();
          return result;
        }
        
       
      }
      AstraPro::reconfigureCallback();
      return result;
    };
  this->set_on_parameters_set_callback(param_change_callback);
}

void AstraPro::reconfigureCallback()
{
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 / publish_rate_)),
    std::bind(&AstraPro::doWork, this)); 

  camera_info_manager::CameraInfoManager c(this);
  if (!camera_info_url_.empty()) {
    RCLCPP_INFO(get_logger(), "camera_info_url exist");
    try {
      c.validateURL(camera_info_url_);
      c.loadCameraInfo(camera_info_url_);
      camera_info_ = c.getCameraInfo();
    } catch (cv::Exception & e) {
      RCLCPP_ERROR(
        this->get_logger(), "camera calibration failed to load: %s %s %s %i",
        e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }
  } else {
    RCLCPP_INFO(get_logger(), "no camera_info_url exist");
  }
}

void AstraPro::doWork()
{
 
  astra_update();
  std::cout << "Timer interrupt" << std::endl;
  return;

  // Transform the image.
  try {
    if (cap_.isOpened()) {
      if (!cap_.read(image_)) {
        cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
      }
    }
    if (flip_image_) {
      cv::flip(image_, image_, flip_value_);
    }

    sensor_msgs::msg::Image::SharedPtr out_img =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_).toImageMsg();
    out_img->header.frame_id = frame_id_;
    out_img->header.stamp = rclcpp::Clock().now();
    camera_info_.header.frame_id = out_img->header.frame_id;
    camera_info_.header.stamp = out_img->header.stamp;

    pub_.publish(*out_img, camera_info_);
  } catch (cv::Exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Image processing error: %s %s %s %i",
      e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
  }
}

void AstraPro::onInit()
{
  RCLCPP_INFO(this->get_logger(), "File name for publishing image is : %s", filename_.c_str());
  
  
  RCLCPP_INFO(
    this->get_logger(),
    "Flip horizontal image is : %s", ((flip_horizontal_) ? "true" : "false"));
  RCLCPP_INFO(
    this->get_logger(),
    "Flip flip_vertical image is : %s", ((flip_vertical_) ? "true" : "false"));

  // From http://docs.opencv.org/modules/core/doc/operations_on_arrays.html
  // #void flip(InputArray src, OutputArray dst, int flipCode)
  // FLIP_HORIZONTAL == 1, FLIP_VERTICAL == 0 or FLIP_BOTH == -1
  // flip_image_ = true;
  // if (flip_horizontal_ && flip_vertical_) {
  //   flip_value_ = 0;  // flip both, horizontal and vertical
  // } else if (flip_horizontal_) {
  //   flip_value_ = 1;
  // } else if (flip_vertical_) {
  //   flip_value_ = -1;
  // } else {
  //   flip_image_ = fFile name for alse;
  // }

  // camera_info_.width = image_.cols;
  // camera_info_.height = image_.rows;
  // camera_info_.distortion_model = "plumb_bob";
  // camera_info_.d = {0, 0, 0, 0, 0};
  // camera_info_.k = {1, 0, static_cast<float>(camera_info_.width / 2), 0, 1,
  //   static_cast<float>(camera_info_.height / 2), 0, 0, 1};
  // camera_info_.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  // camera_info_.p = {1, 0, static_cast<float>(camera_info_.width / 2), 0, 0, 1,
  //   static_cast<float>(camera_info_.height / 2), 0, 0, 0, 1, 0};

  // timer_ = this->create_wall_timer(
  //   std::chrono::milliseconds(static_cast<int>(1000 / publish_rate_)),
  //   std::bind(&ImagePublisher::doWork, this));

    std::cout << "Camera init begin" << std::endl;


    astra::initialize();

    set_key_handler();
    astra::StreamSet streamSet;
    astra::StreamReader reader = streamSet.create_reader();
    //New
    FrameListener listener;
    reader.stream<astra::ColorStream>().start();
    auto depthStream = reader.stream<astra::DepthStream>();
    depthStream.start();

    reader.add_listener(listener);
    std::cout << "Camera init done" << std::endl;

    //reader.remove_listener(listener);
    //astra::terminate();

}

}  // namespace image_publisher

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(astra_pro::AstraPro)
