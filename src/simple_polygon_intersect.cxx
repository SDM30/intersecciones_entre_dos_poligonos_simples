#include <algorithm>
#include <cmath>
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
Check if the source or target is inside the polygon
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

bool already_inserted(const TPoints& points, const TPoint& point) {
    return std::find(points.begin(), points.end(), point) != points.end();
}

void sort_points_around_centroid(TPoints& points) {
    if (points.size() < 3) {
        return;
    }

    // Compute centroid as the average of all vertices.
    TPoint pivot;
    long double cx = 0.0L;
    long double cy = 0.0L;
    for (const auto& p : points) {
        cx += static_cast<long double>(p.x());
        cy += static_cast<long double>(p.y());
    }
    cx /= static_cast<long double>(points.size());
    cy /= static_cast<long double>(points.size());

    pivot = {cx, cy};
    // Sort points by polar angle around the centroid.
    std::sort(
      points.begin(), points.end(),
      [pivot](const TPoint& a, const TPoint& b) {
          // atan2 gives each point a polar angle around the centroid; sorting
          // angles in ascending order traverses vertices counterclockwise.
          const long double angle_a =
            std::atan2(static_cast<long double>(a.y()) - pivot.y(), static_cast<long double>(a.x()) - pivot.x());
          const long double angle_b =
            std::atan2(static_cast<long double>(b.y()) - pivot.y(), static_cast<long double>(b.x()) - pivot.x());
          return angle_a < angle_b;
      }
    );
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
                if (!already_inserted(intersection, inter.second)) {
                    intersection.push_back(inter.second);
                }
                inter_a1 = seg_a;
                inter_b1 = seg_b;
            }
        }       
    }

    if (is_unassigned(inter_a1) && is_unassigned(inter_b1)) {
        std::cout << "There is no intersection between polygon A and B" << std::endl;
        return intersection;
    }

    // Starting pos either the src or target of inter_a1
    auto start_pair = in_polygon(inter_a1, poly_b);
    if (!start_pair.first) {
        // Both endpoints are outside of pol b
        return intersection;
    }
    TPoint p_start = start_pair.second;
    if (!already_inserted(intersection, p_start)) {
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
            if (!already_inserted(intersection, p_next)) {
                intersection.push_back(p_next);
            }
        }

        p_curr = p_next;
    }

    // Starting pos either the src or target of inter_b1
    start_pair = in_polygon(inter_b1, poly_a);
    if (!start_pair.first) {
        return intersection;
    }
    p_start = start_pair.second;
    if (!already_inserted(intersection, p_start)) {
        intersection.push_back(p_start);
    }

    p_curr = p_start;
    first_step = true;
    auto next_pair = get_next_point(p_curr, poly_b);
    if (!next_pair.first) return intersection;
    TPoint p_next = next_pair.second;

    while (p_next != p_start)
    {
        next_pair = get_next_point(p_curr, poly_b);
        if (!next_pair.first) break;

        p_next = next_pair.second;
        //Full lap with no second intersection
        if (!first_step && p_next == p_start) {
          break;
        }
        first_step = false;

        if (in_polygon(p_next, poly_a).first) {
            if (!already_inserted(intersection, p_next)) {
                intersection.push_back(p_next);
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
    sort_points_around_centroid(intersection);

    if (intersection.size() >= 1) {
        std::cout << "Points saved in " << argv[3] << std::endl;
        pujCGAL::IO::save(argv[3], intersection.begin(), intersection.end());
    }

    TKernel::FT area_inter = 0;
    if (intersection.size() >= 3) {
        TPolygon inter_polygon;
        for (const auto & p : intersection) {
            inter_polygon.push_back(p);
        }

        area_inter = CGAL::abs(inter_polygon.area());
    }

    TKernel::FT area_a = CGAL::abs(poly_a.area());
    TKernel::FT area_b = CGAL::abs(poly_b.area());
    TKernel::FT porcentaje = 0;
    if ((area_inter + area_a + area_b) != 0) {
        porcentaje = CGAL::abs(area_inter * 100 / (area_inter + area_a + area_b));
    }

    std::string metrics_fname = std::string(argv[3]) + "_metrics.txt";
    pujCGAL::IO::save_metrics(
      metrics_fname,
      static_cast<long double>(area_a),
      static_cast<long double>(area_b),
      static_cast<long double>(area_inter),
      static_cast<long double>(porcentaje)
    );

    return 0;
}
