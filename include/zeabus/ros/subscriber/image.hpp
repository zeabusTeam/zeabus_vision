// FILE			: image.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 24 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <mutex>
#include    <iostream>

#include    <ros/ros.h>
#include    <std_msgs/Header.h>

#include    <cv_bridge/cv_bridge.h>
#include    <image_transport/image_transport.h>

#include    <opencv2/core/mat.hpp>

#ifndef _ZEABUS_ROS_SUBSCRIBER_IMAGE_HPP__
#define _ZEABUS_ROS_SUBSCRIBER_IMAGE_HPP__

namespace zeabus_ros
{

namespace subscriber
{

    class Image
    {

        public:
            Image( image_transport::ImageTransport* ptr_image_handle ); 
            
            image_transport::Subscriber subscriber;
            void callback( const sensor_msgs::ImageConstPtr& msg );
            void setup_variable( std::mutex* ptr_mutex_data,
                    cv::Mat* ptr_mat_data,
                    std_msgs::Header* ptr_header_data );
            void setup_subscriber( std::string topic , 
                    std::string type_decode = "rgb8" ,
                    unsigned int queue_size = 1 );

        protected:
            cv::Mat* ptr_mat_data;
            std::string type_decode;
            std::mutex* ptr_mutex_data;
            std_msgs::Header* ptr_header_data;
            image_transport::ImageTransport* ptr_image_handle;

    };

} // namespace subscriber

} // namespace zeabus_ros

#endif // _ZEABUS_ROS_SUBSCRIBER_IMAGE_HPP__
