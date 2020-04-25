// FILE			: convert.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 29 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <iostream>
#include    <opencv2/core/types.hpp>

#ifndef _ZEABUS_OPENCV_CONVERT_HPP__
#define _ZEABUS_OPENCV_CONVERT_HPP__

namespace zeabus_opencv
{

namespace convert
{

    const double ratio = 37.795275591; // cm * ration = pixels

    void to_cm( double* data );
    void to_m( double* data );
    double to_cm( const double data);
    double to_m( const double data);

    double to_pixel( const double data);
    void to_pixel( double* data );

} // namespace convert

} // namespace zeabus_opencv

#endif
