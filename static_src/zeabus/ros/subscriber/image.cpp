// FILE			: image.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, March 29 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <zeabus/ros/subscriber/image.hpp>

namespace zeabus_ros
{

namespace subscriber
{

    Image::Image( image_transport::ImageTransport* ptr_image_handle )
    {
        this->ptr_image_handle = ptr_image_handle;
    }

    void Image::callback( const sensor_msgs::ImageConstPtr& msg )
    {
        this->ptr_mutex_data->lock();
        *( this->ptr_mat_data ) = cv_bridge::toCvShare( msg , this->type_decode )->image;
        *( this->ptr_header_data ) = msg->header;
        this->ptr_mutex_data->unlock();
    }

    void Image::setup_variable( std::mutex* ptr_mutex_data,
            cv::Mat* ptr_mat_data,
            std_msgs::Header* ptr_header_data )
    {
        this->ptr_mutex_data = ptr_mutex_data;
        this->ptr_mat_data = ptr_mat_data;
        this->ptr_header_data = ptr_header_data;
    }

    void Image::setup_subscriber( std::string topic , 
            std::string type_decode , 
            unsigned int queue_size )
    {
        this->type_decode = type_decode;
        this->subscriber = this->ptr_image_handle->subscribe( topic , 
                queue_size , 
                &zeabus_ros::subscriber::Image::callback , 
                this );
    }

}

}
