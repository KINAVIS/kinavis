// Inward offset of polygons with holes along their straight skeleton, by CGAL:
// the reference for `Region::inset`.
//
// Input, one region per block:
//     ring x y x y ...      outer boundary, then each hole, any orientation
//     offset d              distance, metres; ends the block
// Output, per block: `count n` with the number of polygons left, then for
// each `area a holes k` and its rings as `ring x y ...`, outer first.
//
// Coordinates are moved to the first vertex before CGAL sees them and back
// after, as the Rust side does.
//
//     c++ -std=c++17 -O2 -I/opt/homebrew/include -L/opt/homebrew/lib \
//         tools/skeleton_offset.cpp -lgmp -lmpfr -o skeleton_offset

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/create_offset_polygons_from_polygon_with_holes_2.h>

#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point = K::Point_2;
using Polygon = CGAL::Polygon_2<K>;
using PolygonWithHoles = CGAL::Polygon_with_holes_2<K>;

int main() {
    std::vector<std::vector<std::pair<double, double>>> rings;
    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream words(line);
        std::string tag;
        words >> tag;
        if (tag == "ring") {
            std::vector<std::pair<double, double>> ring;
            double x, y;
            while (words >> x >> y) ring.emplace_back(x, y);
            rings.push_back(ring);
            continue;
        }
        if (tag != "offset") continue;
        double distance;
        words >> distance;
        const double ox = rings.at(0).at(0).first;
        const double oy = rings.at(0).at(0).second;
        auto build = [&](const std::vector<std::pair<double, double>>& ring, bool counter_clockwise) {
            Polygon polygon;
            for (auto [x, y] : ring) polygon.push_back(Point(x - ox, y - oy));
            if (polygon.is_counterclockwise_oriented() != counter_clockwise) polygon.reverse_orientation();
            return polygon;
        };
        PolygonWithHoles region(build(rings[0], true));
        for (size_t i = 1; i < rings.size(); ++i) region.add_hole(build(rings[i], false));
        auto result = CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(distance, region);
        std::printf("count %zu\n", result.size());
        for (const auto& piece : result) {
            double area = CGAL::to_double(piece->outer_boundary().area());
            for (const auto& hole : piece->holes()) area += CGAL::to_double(hole.area());
            std::printf("area %.9f holes %zu\n", area, static_cast<size_t>(piece->number_of_holes()));
            auto print = [&](const Polygon& ring) {
                std::printf("ring");
                for (const auto& p : ring.vertices())
                    std::printf(" %.9f %.9f", CGAL::to_double(p.x()) + ox, CGAL::to_double(p.y()) + oy);
                std::printf("\n");
            };
            print(piece->outer_boundary());
            for (const auto& hole : piece->holes()) print(hole);
        }
        std::fflush(stdout);
        rings.clear();
    }
}
