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

#include "FrameListener.hpp"



using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class AstraPro : public rclcpp::Node
{
  public:
    AstraPro()
    : Node("astra_pro_publisher"), count_(0)
    {
        update_thread = std::thread(&AstraPro::camera_update, this);    
        colorpublisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
        depthpublisher_ = this->create_publisher<sensor_msgs::msg::Image>("depth_raw", 10);
    }   

    void color_send(sensor_msgs::msg::Image data)
    {
        colorpublisher_->publish(data);
    }

    void depth_send(sensor_msgs::msg::Image data)
    {
        depthpublisher_->publish(data);
    }

  private:
    void camera_update()
    {
        astra::initialize();

        set_key_handler();

        astra::StreamSet streamSet;
        astra::StreamReader reader = streamSet.create_reader();
        //New
        FrameListener listener( std::bind(&AstraPro::color_send, this, std::placeholders::_1), 
                                std::bind(&AstraPro::depth_send, this, std::placeholders::_1));
        reader.stream<astra::ColorStream>().start();


        auto depthStream = reader.stream<astra::DepthStream>();
        depthStream.start();


        reader.add_listener(listener);


        do{
            astra_update();         
        } while (true);
        
        reader.remove_listener(listener);
        
            
        astra::terminate();
        
    }
  
    size_t count_;
    std::thread update_thread;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr colorpublisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthpublisher_;
    
  };
