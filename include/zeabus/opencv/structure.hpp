// FILE			: structure.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 27 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <opencv2/core/types.hpp>
#include    <cmath>

#ifndef _ZEABUS_OPENCV_STRUCTURE_HPP__
#define _ZEABUS_OPENCV_STRUCTURE_HPP__

namespace zeabus_opencv
{

namespace structure
{

    struct Circle
    {
        cv::Point2f center;
        float radius; 

        Circle( float x , float y , float radius );
        Circle( cv::Point2f center , float radius );

        Circle& operator=( const cv::Point2f center );
        Circle& operator=( const float radius );
        Circle& operator=( const Circle& other );
        Circle& operator=( const float* data );

    };

    bool operator<( const Circle& lhs , const Circle& rhs );
    bool operator>( const Circle& lhs , const Circle& rhs );
    float operator-( const Circle& lhs , const Circle& rhs );

    struct LineRect
    {
        cv::Point2f bl; // bottom_left
        cv::Point2f tl; // top_left
        cv::Point2f tr; // top_right
        cv::Point2f br; // bottom_right

        std::vector< cv::Point2f > get_vector();
    };

} // namespace structure

} // namespace zeabus_opencv

#endif // _ZEABUS_OPENCV_STRUCTURE_HPP__
