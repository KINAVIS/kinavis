/*
 * Writes tests/data/reeds_shepp_reference.txt from the Reeds–Shepp core of
 * OMPL 1.7.0 (https://github.com/ompl/ompl, BSD-3-Clause;
 * src/ompl/base/spaces/src/ReedsSheppStateSpace.cpp, by Mark Moll).
 *
 * The core is the anonymous namespace of that file and its table of path
 * types; the state-space classes around it are not needed. It is cut out at
 * build time and compiled against the minimal declarations below:
 *
 *   F=OMPL/src/ompl/base/spaces/src/ReedsSheppStateSpace.cpp
 *   sed -n '/^namespace$/,/^};$/p' $F > rs_core.inc
 *   c++ -std=c++17 -O2 -I/opt/homebrew/include -I. \
 *       tools/reeds_shepp_reference.cpp -o reeds_shepp_reference
 *   ./reeds_shepp_reference > tests/data/reeds_shepp_reference.txt
 *
 * Random start and end poses within 100 m, radii from 1 to 20 m, a third of
 * them close together, then turns from the end of a pass onto a parallel
 * pass heading back. Frame of the library: x east, y north, heading
 * counter-clockwise from east, radians. Columns: x0 y0 heading0 x1 y1
 * heading1 radius type length0..length4, lengths signed (negative in
 * reverse) in metres; types 0..17 index the library's table.
 *
 * Only the values are kept in the repository; none of the library is.
 */
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <boost/math/constants/constants.hpp>
#include <limits>

namespace ompl::base {
struct State;
struct ReedsSheppStateSpace {
    enum ReedsSheppPathSegmentType { RS_NOP = 0, RS_LEFT = 1, RS_STRAIGHT = 2, RS_RIGHT = 3 };
    static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];
    struct ReedsSheppPath {
        ReedsSheppPath(const ReedsSheppPathSegmentType *type = reedsSheppPathType[0],
                       double t = std::numeric_limits<double>::max(), double u = 0., double v = 0.,
                       double w = 0., double x = 0.)
          : type_(type), length_{t, u, v, w, x},
            totalLength_(std::fabs(t) + std::fabs(u) + std::fabs(v) + std::fabs(w) + std::fabs(x))
        {
        }
        double length() const { return totalLength_; }
        const ReedsSheppPathSegmentType *type_;
        double length_[5];
        double totalLength_;
    };
};
}  // namespace ompl::base

using namespace ompl::base;

#include "rs_core.inc"

static uint64_t state = 20261021;

/* splitmix64, as in the Rust tests. */
static double next()
{
    uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return (double)((z ^ (z >> 31)) >> 11) / 9007199254740992.0;
}

/* The library's own reduction to the start frame, in units of the radius. */
static void write(const double q0[3], const double q1[3], double rho)
{
    double dx = q1[0] - q0[0], dy = q1[1] - q0[1], c = std::cos(q0[2]), s = std::sin(q0[2]);
    double x = c * dx + s * dy, y = -s * dx + c * dy, phi = q1[2] - q0[2];
    ReedsSheppStateSpace::ReedsSheppPath path = reedsShepp(x / rho, y / rho, phi);
    int type = (int)(path.type_ - ReedsSheppStateSpace::reedsSheppPathType[0]) / 5;
    printf("%.12f %.12f %.15f %.12f %.12f %.15f %.12f %d", q0[0], q0[1], q0[2], q1[0], q1[1],
           q1[2], rho, type);
    for (double length : path.length_) {
        printf(" %.12f", length * rho);
    }
    printf("\n");
}

int main()
{
    const double tau = 6.283185307179586;
    printf("# x0 y0 heading0 x1 y1 heading1 radius type length0 length1 length2 length3 length4\n");
    for (int i = 0; i < 3000; ++i) {
        double spread = (i % 3 == 0) ? 15.0 : 100.0;
        double q0[3] = { spread * (next() - 0.5), spread * (next() - 0.5), tau * next() };
        double q1[3] = { spread * (next() - 0.5), spread * (next() - 0.5), tau * next() };
        write(q0, q1, 1.0 + 19.0 * next());
    }
    /* Headland turns: a pass ends heading north, the next is `spacing` to
       the side, joined abreast or a little behind, heading south. */
    for (int i = 0; i < 600; ++i) {
        double rho = 3.0 + 9.0 * next();
        double spacing = (next() - 0.5) * 6.0 * rho;
        double behind = -2.0 * rho * next();
        double q0[3] = { 0.0, 0.0, tau / 4.0 };
        double q1[3] = { spacing, behind, -tau / 4.0 };
        write(q0, q1, rho);
    }
    return 0;
}
