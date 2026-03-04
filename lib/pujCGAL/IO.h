#ifndef __pujCGAL__IO__h__
#define __pujCGAL__IO__h__
#include <string>

namespace pujCGAL
{
  namespace IO
  {
    template< class TSegmentsIt >
    bool read( const std::string& fname, TSegmentsIt sIt );

    template< class TPointsIt >
    bool save( const std::string& fname, TPointsIt pB, TPointsIt pE );

    bool save_metrics(
      const std::string& fname,
      long double area_a,
      long double area_b,
      long double area_intersection,
      long double percentage
      );

  } // end namespace
} // end namespace

#include <pujCGAL/IO.hxx>

#endif // __pujCGAL__IO__h__

// eof - IO.h
