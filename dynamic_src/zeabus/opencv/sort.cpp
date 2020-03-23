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

namespace zeabus_opencv
{

namespace sort
{

    template< typename _Tp >
    void center( std::vector< cv::Point_< _Tp > >* ptr_source )
    {
        std::vector< cv::Point_< _Tp > > vec_answer;
        std::vector< double > size_answer;
        for( unsigned int run = 0 ; run < ptr_source->size() ; run++ )
        {
            double size = zeabus_opencv::operations::size( ptr_source->at( run ) );
            unsigned int limit = vec_answer.size() + 1;
            for( unsigned int find = 0 ; find < limit ; find++ )
            {
                if( find == limit - 1 )
                {
                    vec_answer.push_back( ptr_source->at( run ) );
                    size_answer.push_back( size );
                }
                else if( size < size_answer.at( find ) )
                {
                    vec_answer.insert( vec_answer.begin() + find , ptr_source->at( run ) );
                    size_answer.insert( size_answer.begin() + find , size );
                    break;
                }
                else
                {
                    ;
                }
            }
        } 
        ptr_source->swap( vec_answer );
        vec_answer.clear();
        size_answer.clear();
    }

} // opencv

} // zeabus
