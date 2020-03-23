// FILE			: convert.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 29 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <zeabus/opencv/convert.hpp>

namespace zeabus_opencv
{

namespace convert
{

    void to_cm( double* data )
    {
        *data = (*data) / ratio;
    }

    void to_m( double* data )
    {
        *data = (*data) / ratio / 100;
    }

    double to_cm( const double data)
    {
        return data/ratio;
    }

    double to_m( const double data)
    {
        return data/ratio/100;
    }

    double to_pixel( const double data)
    {
        return data*ratio;
    }

    void to_pixel( double* data )
    {
        *data = (*data) * ratio;
    }

} // namespace convert

} // namespace zeabus_opencv
