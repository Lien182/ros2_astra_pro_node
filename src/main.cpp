// This file is part of the Orbbec Astra SDK [https://orbbec3d.com]
// Copyright (c) 2015-2017 Orbbec 3D
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Be excellent to each other.


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"


#include <astra/astra.hpp>
#include <cstdio>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <key_handler.h>


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



using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class AstraPro : public rclcpp::Node
{
  public:
    AstraPro()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&AstraPro::timer_callback, this));



    astra::initialize();

    set_key_handler();

    astra::StreamSet streamSet;
    astra::StreamReader reader = streamSet.create_reader();

    //New
    FrameListener listener;
    reader.stream<astra::ColorStream>().start();
    //Endnew

    //DepthFrameListener depthListener;

    auto depthStream = reader.stream<astra::DepthStream>();
    depthStream.start();


    reader.add_listener(listener);


    /*
    do{
        astra_update();
    } while (shouldContinue);

    reader.remove_listener(listener);
    
    	

    astra::terminate();
    */

    }

  private:
    void timer_callback()
    {
      auto message = sensor_msgs::msg::Image();

      
      //message.data = "Hello, world! " + std::to_string(count_++);
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    size_t count_;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AstraPro>());
    rclcpp::shutdown();
    return 0;
  }
