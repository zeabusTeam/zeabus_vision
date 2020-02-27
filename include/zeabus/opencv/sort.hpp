// FILE			: sort.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2020, Febuary 27 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE
// This implement on vector of cpp

// MACRO SET

// MACRO CONDITION

#include    <vector>
#include    <opencv2/core/types.hpp>

#ifndef _ZEABUS_OPENCV_SORT_HPP__
#define _ZEABUS_OPENCV_SORT_HPP__

namespace zeabus_opencv
{

namespace sort
{

    template< typename types >
    inline void cpp_sort( std::vector< types >* ptr_vector )
    {
        std::stable_sort( ptr_vector->begin() , ptr_vector->end() );
    }

    template< typename _Tp >
    void sort_center( std::vector< cv::Point< _Tp > >* ptr_vector );

} // namespace sort

} // namesapce zeabus_opencv

#include    <zeabus/opencv/sort.cpp>

#endif // _ZEABUS_OPENCV_SORT_HPP__
