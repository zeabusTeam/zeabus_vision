// FILE			: camera_info.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 28 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <iostream>
#include    <opencv2/core/types.hpp>
#include    <opencv2/core/mat.hpp>

#ifndef _ZEABUS_OPENCV_CAMERA_INFO_HPP__
#define _ZEABUS_OPENCV_CAMERA_INFO_HPP__

namespace zeabus_opencv
{

namespace front
{
    const double camera_matrix[ 3 * 3 ] = {
        1403.392497146628,      0,                      965.5521004025143,
        0,                      1402.043155983041,      634.1035439812066,
        0,                      0,                      1
    };
    
    const std::string distortion_model = "plumb_bob";

    const double distortion_coefficients[ 1 * 5 ] = {
        0.08006918973404457,                0.3813407462666498,     
        0.001438037539977991,               -5.205265826661288e-05,
        0
    };
    
    const double rectification[ 3 * 3 ] = {
        1,                      0,                      0,
        0,                      1,                      0,
        0,                      0,                      1
    }; 

    const double projection[ 3 * 4 ] = {
        1608.941040039062,  0,                  966.0538982648359,  0,
        0,                  1603.074584960938,  633.1073181585307,  0,
        0,                  0,                  1,                  0
    };

    const cv::Mat_<double> mat_camera( 3, 3, 
            (double*)&camera_matrix , 
            (size_t)sizeof( double )*3 ); 

    const cv::Mat_<double> mat_distor( 1, 5, 
            (double*)&distortion_coefficients, 
            (size_t)sizeof( double )*5 );

} // namespace front

namespace bottom
{

    const double camera_matrix[ 3 * 3 ] = {
        1936.0,                 0,                      868.0,
        0,                      1936.0,                 608.0,
        0,                      0,                      1
    };
    
    const std::string distortion_model = "plumb_bob";

    const double distortion_coefficients[ 0 * 0 ] = {};
    
    const double rectification[ 3 * 3 ] = {
        1,                      0,                      0,
        0,                      1,                      0,
        0,                      0,                      1
    }; 

    const double projection[ 3 * 4 ] = {
        1608.941040039062,  0,                  966.0538982648359,  0,
        0,                  1603.074584960938,  633.1073181585307,  0,
        0,                  0,                  1,                  0
    };

    const cv::Mat_<double> mat_camera( 3, 3, 
            (double*)&camera_matrix , 
            (size_t)sizeof( double )*3 ); 

    const cv::Mat_<double> mat_distor( 0, 0, 
            (double*)&distortion_coefficients, 
            (size_t)sizeof( double )*0 );

} // namespace bottom

} // namespace zeabus_opencv

#endif // _ZEABUS_OPENCV_CAMERA_INFO_HPP__
