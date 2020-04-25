// FILE			: operations.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 27 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE
// This implement on vector of cpp

// MACRO SET

// MACRO CONDITION

#include    <cmath>

#include    <opencv2/core/types.hpp>

#ifndef _ZEABUS_OPENCV_OPERATIONS_HPP__
#define _ZEABUS_OPENCV_OPERATIONS_HPP__

namespace zeabus_opencv
{

namespace operations
{

    template< typename type_sorce , typename _Tp >
    void pull_center( const std::vector< type_sorce >& source ,
            std::vector< cv::Point_< _Tp > >* ptr_dest )
    {
        ptr_dest->clear();
        for( auto it = source.cbegin() ; it != source.cend() ; it++ )
        {
            ptr_dest->push_back( it->center );
        }
    }

    template< typename _Tp >
    inline double size( const cv::Point_< _Tp >& point )
    {
        return sqrt( pow( point.x , 2 ) + pow( point.y , 2 ) );
    }

    template< typename _Tp >
    inline double ratio( const cv::Size_< _Tp >& size )
    {
        double answer = size.width / size.height;
        if( answer > 1 ) answer = 1.0 / answer;
        return answer;
    }

} // zeabus_opencv

} // operations

#endif //_ZEABUS_OPENCV_OPERATIONS_HPP__
