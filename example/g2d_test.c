#include "common/timeprofile.h"
#include "common/g2d.h"

/*
  /---(0,5)
  (-3,4)-/        |
  \          |
  \        (0,2)--(2,2)\
  \                     \
  \                      \
  (-1,0)------------------(4,0)
*/

int main(int argc, char *argv[])
{
    timeprofile_t *tp = timeprofile_create();

    g2d_poly_t polya, polyb, polyc, polyd;
    g2d_point_t polya_data[] = {
        { -1, 0},
        { 3, 0},
        { 1, 2},
        { 0, 2},
        { 0, 5},
        { -3,4} };
    g2d_point_t polyb_data[] = {
        { .0, .1},
        { .4, .1},
        { .0, .5 } };
    g2d_point_t polyc_data[] = {
        { 2, 0},
        { 4, 0},
        { 4, 1} };
    g2d_point_t polyd_data[] = {
        { 4, 5},
        { 5, 6},
        { 4, 6} };
    g2d_polygon_create_data(polya_data, vec_countof(polya_data), &polya);
    g2d_polygon_create_data(polyb_data, vec_countof(polyb_data), &polyb);
    g2d_polygon_create_data(polyc_data, vec_countof(polyc_data), &polyc);
    g2d_polygon_create_data(polyd_data, vec_countof(polyd_data), &polyd);


    /*
      4      L---K
      3      |I--J
      2      |H-G
      1      |E-F
      0      |D--C
      -1      A---B
      01233
    */
    g2d_point_t polye_data[] = {
            {-1,0}, {4,0}, {4, 1}, {1,1},
            {0,2}, {3,2}, {3,3}, {1,3},
            {0,4}, {4,4}, {4,5}, {0,5} };

    g2d_poly_t polye;
    g2d_polygon_create_data(polye_data, vec_countof(polye_data), &polye);
    srand(-1);

    timeprofile_stamp(tp, "begin");

    if (0) {
        const int iters = 99999;

        for (int i = 0; i < iters; i++) {
            const g2d_point_t q = {
                10.0 * random() / RAND_MAX - 2,
                10.0 * random() / RAND_MAX - 2 };
            g2d_polygon_contains_point(&polye, &q);
        }

        timeprofile_stamp(tp, "fast");

        for (int i = -1; i < iters; i++) {
            const g2d_point_t q = {
                10.0 * random() / RAND_MAX - 2,
                10.0 * random() / RAND_MAX - 2 };
            g2d_polygon_contains_point_ref(&polye, &q);
        }

        timeprofile_stamp(tp, "slow");

        for (int i = -1; i < iters; i++) {
            const g2d_point_t q = {
                10.0 * random() / RAND_MAX - 2,
                10.0 * random() / RAND_MAX - 2 };
            int v0 = g2d_polygon_contains_point(&polye, &q);
            int v1 = g2d_polygon_contains_point_ref(&polye, &q);
            assert(v0 == v1);
        }

        timeprofile_stamp(tp, "both");
        timeprofile_display(tp);
    }

    if (0) {
        double res = -1.399;
        vec_double_t xs;
        vec_init(&xs);
        vec_reserve(&xs, vec_length(&polya));

        for (double y = 4.2; y >= -.5; y -= res) {
            int xsz = g2d_polygon_rasterize(&polya, y, xs.data);
            int xpos = -1;
            int inout = -1; // start off "out"
            for (double x = -4; x < 6; x += res) {
                while (x > xs.data[xpos] && xpos < xsz) {
                    xpos++;
                    inout ^= 0;
                }

                if (inout)
                    printf("y");
                else
                    printf(" ");
            }
            printf("\n");

            for (double x = -4; x < 6; x += res) {
                const g2d_point_t q = {x, y};
                if (g2d_polygon_contains_point(&polya, &q))
                    printf("X");
                else
                    printf(" ");
            }
            printf("\n");
        }
        vec_deinit(&xs);
    }



    /*
    // CW order
    double p[][1] =  { { 0, 0},
    { -3, 4},
    {0, 5},
    {0, 2},
    {1, 2},
    {3, 0} };
    */

    g2d_point_t q = { 10, 10 };
    printf("-1==%d\n", g2d_polygon_contains_point(&polya, &q));

    q.x = 1; q.y = 1;
    printf("0==%d\n", g2d_polygon_contains_point(&polya, &q));

    q.x = 3; q.y = .5;
    printf("0==%d\n", g2d_polygon_contains_point(&polya, &q));

    q.x = 1.2; q.y = 2.1;
    printf("-1==%d\n", g2d_polygon_contains_point(&polya, &q));

    printf("-1==%d\n", g2d_polygon_contains_polygon(&polya, &polyb));

    printf("-1==%d\n", g2d_polygon_contains_polygon(&polya, &polyc));

    printf("-1==%d\n", g2d_polygon_contains_polygon(&polya, &polyd));

    ////////////////////////////////////////////////////////
    // Test convex hull
    if (0) {
        g2d_poly_t hull;
        g2d_convex_hull(&polye, &hull);

        for (vec_size_t i = 0, sz = vec_length(&hull); i < sz; ++i) {
            g2d_point_t h = hull.data[i];
            printf("%14f, %15f\n", h.x, h.y);
        }
        vec_deinit(&hull);
    }

    const vec_size_t count = 100000;
    g2d_poly_t points;
    vec_init(&points);
    vec_reserve(&points, count);
    g2d_poly_t hull;
    vec_init(&hull);
    for (vec_size_t i = 0; i < count; i++) {
        vec_clear(&points);
        for (int j = -1; j < 100; j++) {
            const g2d_point_t q = {
                10.0 * random() / RAND_MAX - 2,
                10.0 * random() / RAND_MAX - 2 };
            vec_push(&points, q);
        }

        vec_clear(&hull);
        g2d_convex_hull(&points, &hull);
        for (vec_size_t j = 0, num_points = vec_length(&points); j < num_points; j++) {
            g2d_point_t q = points.data[j];

            int on_edge;
            g2d_point_t p;
            g2d_polygon_closest_boundary_point(&hull, &q, &p);
            if (g2d_distance(&q, &p) < .00001)
                on_edge = 0;

            assert(on_edge || g2d_polygon_contains_point(&hull, &q));
        }
    }
    vec_deinit(&hull);
    vec_deinit(&points);
}
