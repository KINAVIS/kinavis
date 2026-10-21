/*
 * Writes tests/data/dubins_reference.txt from Andrew Walker's Dubins
 * library (https://github.com/AndrewWalker/Dubins-Curves, MIT; the C core
 * of the `dubins` Python package 1.0.1, dubins/src/dubins.c).
 *
 * Random start and end poses within 100 m, radii from 1 to 20 m, and the
 * shortest path between them: its word and the length of each piece.
 * Frame of the library: x east, y north, heading counter-clockwise from
 * east, radians. Columns: x0 y0 heading0 x1 y1 heading1 radius word
 * length0 length1 length2, lengths in metres; words 0..5 are LSL LSR RSL
 * RSR RLR LRL.
 *
 *   cc -O2 -I DUBINS/dubins/include tools/dubins_reference.c \
 *       DUBINS/dubins/src/dubins.c -lm -o dubins_reference
 *   ./dubins_reference > tests/data/dubins_reference.txt
 *
 * Only the values are kept in the repository; none of the library is.
 */
#include <stdint.h>
#include <stdio.h>

#include "dubins.h"

static uint64_t state = 20261017;

/* splitmix64, as in the Rust tests. */
static double next(void)
{
    uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return (double)((z ^ (z >> 31)) >> 11) / 9007199254740992.0;
}

int main(void)
{
    const double tau = 6.283185307179586;
    printf("# x0 y0 heading0 x1 y1 heading1 radius word length0 length1 length2\n");
    for (int i = 0; i < 3000; ++i) {
        /* A third of the cases close together, where the CCC words win. */
        double spread = (i % 3 == 0) ? 15.0 : 100.0;
        double q0[3] = { spread * (next() - 0.5), spread * (next() - 0.5), tau * next() };
        double q1[3] = { spread * (next() - 0.5), spread * (next() - 0.5), tau * next() };
        double rho = 1.0 + 19.0 * next();
        DubinsPath path;
        if (dubins_shortest_path(&path, q0, q1, rho) != 0) {
            continue;
        }
        printf("%.12f %.12f %.15f %.12f %.12f %.15f %.12f %d %.12f %.12f %.12f\n",
               q0[0], q0[1], q0[2], q1[0], q1[1], q1[2], rho, (int)dubins_path_type(&path),
               dubins_segment_length(&path, 0), dubins_segment_length(&path, 1),
               dubins_segment_length(&path, 2));
    }
    return 0;
}
