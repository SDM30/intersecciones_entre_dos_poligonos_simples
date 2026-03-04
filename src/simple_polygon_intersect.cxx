#include <algorithm>
#include <iostream>
#include <vector>

#include <CGAL/Cartesian.h>
#include <CGAL/intersections.h>
#include <CGAL/Polygon_2.h>

#include <pujCGAL/IO.h>

using TReal = long double;
using TKernel = CGAL::Cartesian< TReal >;
using TPolygon = CGAL::Polygon_2<TKernel>;
using TPoint = TKernel::Point_2;
using TSegment = TKernel::Segment_2;
using TSegments = std::vector< TSegment >;
using TPoints = std::vector< TPoint >;

std::pair< bool, TPoint > intersect( const TSegment& a, const TSegment& b ) {
  auto result = CGAL::intersection( a, b );
  // r tiene tres resultados:
  // apuntador nulo
  // punto de interseccion
  if( result )
  {
    if( const TPoint* p = std::get_if< TPoint >( &*result ) )
      return( std::make_pair( true, *p ) );
    else
      return( std::make_pair( false, TPoint( ) ) );
  }
  else
    return( std::make_pair( false, TPoint( ) ) );
}

TPolygon polygon_from_segments(const TSegments& segments) {

    TPolygon poly;
    if (segments.empty()) {
        return poly;
    }

    poly.push_back(segments[0].source());

    for (const auto& seg : segments) {
        // Avoid duplicating the first vertex in closed polylines.
        if (seg.target() != poly[0]) {
            poly.push_back(seg.target());
        }
    }

    return poly;
}

/*
Check if the source or targert is inside the polygon
*/ 
std::pair<bool, TPoint> in_polygon(const TSegment& segment, const TPolygon& poly) {
    TPoint p_src = segment.source();
    TPoint p_trgt = segment.target();
    bool src_inside = true;
    bool trgt_inside = true;

    for (const auto& seg : poly.edges()) {
        CGAL::Orientation side_src = CGAL::orientation(seg.source(), seg.target(), p_src);
        CGAL::Orientation side_trgt = CGAL::orientation(seg.source(), seg.target(), p_trgt);

        // LEFT_TURN or COLLINEAR are accepted as inside/on-boundary.
        if (side_src == CGAL::RIGHT_TURN) src_inside = false;
        if (side_trgt == CGAL::RIGHT_TURN) trgt_inside = false;
    }

    if (src_inside) {
        return {true, p_src};
    }
    if (trgt_inside) {
        return {true, p_trgt};
    }
    return {false, TPoint()};
}

std::pair<bool, TPoint> in_polygon(const TPoint& p, const TPolygon& poly) {
    for (const auto& seg : poly.edges()) {
        CGAL::Orientation side_src = CGAL::orientation(seg.source(), seg.target(), p);
        if (side_src == CGAL::RIGHT_TURN)
            return {false, TPoint()};
    }

    return {true, p};
}

bool is_unassigned(const TSegment& seg) {
    return seg.source() == seg.target();
}

// TODO: implement nextPoint
std::pair<bool, TPoint> get_next_point(const TPoint& src, const TPolygon& poly) {
    auto it = std::find_if(poly.edges_begin(), poly.edges_end(),
                           [&](const TSegment& seg) { return seg.source() == src; });

    if (it != poly.edges_end()) return {true, it->target()};
    return {false, TPoint{}};
}

TPoints get_intersection(const TPolygon& poly_a, const TPolygon& poly_b) {
    TPoints intersection;
    // Intersection segments
    TSegment inter_a1, inter_b1, inter_a2, inter_b2;

    std::pair<bool,TPoint> inter;
    for(const TSegment& seg_a: poly_a.edges()){
        for(const TSegment& seg_b: poly_b.edges()){
            inter = intersect(seg_a,seg_b);
            if(inter.first) {
                intersection.push_back(inter.second);
                inter_a1 = seg_a;
                inter_b1 = seg_b;
                break;
            }
        }
        if (inter.first) {
            break;
        }        
    }

    if (is_unassigned(inter_a1) && is_unassigned(inter_b1)) {
        std::cout << "There is no intersection between polygon A and B" << std::endl;
        return intersection;
    }

    // Starting pos either the src or target of inter_a1
    auto start_pair = in_polygon(inter_a1, poly_b);
    if (!start_pair.first) {
        return intersection;
    }
    TPoint p_start = start_pair.second;
    if (intersection.empty() || p_start != intersection[0]) {
        intersection.push_back(p_start);
    }
    TPoint p_curr = p_start;
    bool first_step = true;
    while (is_unassigned(inter_a2) && is_unassigned(inter_b2))
    {
        auto next_pair = get_next_point(p_curr, poly_a);
        if (!next_pair.first) break;

        TPoint p_next = next_pair.second;

        //Full lap with no second intersection
        if (!first_step && p_next == p_start) {
            break;
        }
        first_step = false;

        if (in_polygon(p_next, poly_b).first) {
            intersection.push_back(p_next);
        } else {
            TSegment seg_a(p_curr, p_next);
            bool found = false;

            for(const auto& seg_b : poly_b.edges()) {
                auto inter = intersect(seg_a, seg_b);
                if (inter.first && inter.second != intersection[0]) {
                    intersection.push_back(inter.second);
                    inter_a2 = seg_a;
                    inter_b2 = seg_b;
                    found = true;
                    break;
                }
            }
        }

        p_curr = p_next;
    }
    
    return intersection;
}

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cout << argv[0] << " polygon_a polygon_b intersection" << std::endl;
        return 1;
    }

    TSegments segments_a, segments_b;

    pujCGAL::IO::read(argv[1], std::back_inserter(segments_a));
    pujCGAL::IO::read(argv[2], std::back_inserter(segments_b));

    TPolygon poly_a = polygon_from_segments(segments_a);
    TPolygon poly_b = polygon_from_segments(segments_b);

    TPoints intersection = get_intersection(poly_a, poly_b);
    for (const auto& p: intersection) {
        std::cout << p << std::endl;
    }

    return 0;
}
