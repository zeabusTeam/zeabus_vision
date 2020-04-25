// FILE			: structure.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 27 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <zeabus/opencv/structure.hpp>

namespace zeabus_opencv
{

namespace structure
{

    Circle::Circle( float x , float y , float radius )
    {
        this->center.x = x;
        this->center.y = y;
        this->radius = radius;
    }

    Circle::Circle( cv::Point2f center , float radius )
    {
        this->center = center;
        this->radius = radius;
    }

    Circle& Circle::operator=( const cv::Point2f center )
    {
        this->center = center;
        return *this;
    }

    Circle& Circle::operator=( const float radius )
    {
        this->radius = radius;
        return *this;
    }

    Circle& Circle::operator=( const Circle& other )
    {
        if( this != &other )
        {
            this->center = other.center;
            this->radius = other.radius;
        }
        return *this;
    }

    Circle& Circle::operator=( const float* data )
    {
        this->center.x = data[0];
        this->center.y = data[1];
        this->radius = data[2];
        return *this;
    }

    float operator-( const Circle& lhs , const Circle& rhs )
    {
        return sqrt( pow( lhs.center.x - rhs.center.x,2 )+ pow( lhs.center.y - rhs.center.y,2) );  
    }

    bool operator<( const Circle& lhs , const Circle& rhs )
    {
        return lhs.radius < rhs.radius;
    }

    bool operator>( const Circle& lhs , const Circle& rhs )
    {
        return lhs.radius > rhs.radius;
    }

    std::vector< cv::Point2f > LineRect::get_vector()
    {
        std::vector< cv::Point2f > answer;
        answer.push_back( this->bl );
        answer.push_back( this->tl );
        answer.push_back( this->tr );
        answer.push_back( this->br );
        return answer;
    }

} // namespace structure

} // namespace zeabus_opencv

