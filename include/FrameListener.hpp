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

    std::function<void(sensor_msgs::msg::Image)> color_callback;
    std::function<void(sensor_msgs::msg::Image)> depth_callback; 

public:
    FrameListener(std::function<void(sensor_msgs::msg::Image)> _color_callback, std::function<void(sensor_msgs::msg::Image)> _depth_callback )
    {
            this->color_callback = _color_callback;
            this->depth_callback = _depth_callback; 
    }


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


            auto ros_image = sensor_msgs::msg::Image();            
            ros_image.height = (height);
            ros_image.width = (width);
            ros_image.encoding = ("mono16");
            ros_image.step = (ros_image.width * 2);
            
            size_t size = ros_image.step * height;
            ros_image.data.resize(size);
            memcpy((void*)&ros_image.data[0],(void*)depthbuffer_.get() , size);
            depth_callback(ros_image);
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


            auto ros_image = sensor_msgs::msg::Image();            
            ros_image.height = (colorlastHeight_);
            ros_image.width = (colorlastWidth_);
            ros_image.encoding = ("rgb8");
            ros_image.step = (ros_image.width * 3);
            
            size_t size = ros_image.step * colorlastHeight_;
            ros_image.data.resize(size);
            memcpy((void*)&ros_image.data[0],(void*)colorbuffer_.get() , size);
             color_callback(ros_image);

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
