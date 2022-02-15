/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.
This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "g2d.h"
#include "common/math_util.h"

#ifdef _WIN32
static inline long int random(void)
{
        return rand();
}
#endif


int g2d_polygon_create_empty(g2d_poly_t *poly)
{
    vec_init(poly);
    return vec_reserve(poly, 32);
}

void g2d_polygon_add(g2d_poly_t *poly, const g2d_point_t *v)
{
    vec_push(poly, *v);
}

int g2d_polygon_create_data(const g2d_point_t *v, int sz, g2d_poly_t *poly)
{
    vec_init(poly);
    if (VEC_OK != vec_reserve(poly, sz)) {
        return -1;
    }

    for (int i = 0; i < sz; i++)
        vec_push(poly, v[i]);

    return 0;
}

int g2d_polygon_create_zeros(int sz, g2d_poly_t *poly)
{
    vec_init(poly);
    g2d_point_t zero = { 0, 0 };

    for (int i = 0; i < sz; i++)
        vec_push(poly, zero);

    return 0;
}

void g2d_polygon_make_ccw(g2d_poly_t *poly)
{
    // Step one: we want the points in counter-clockwise order.
    // If the points are in clockwise order, we'll reverse them.
    double total_theta = 0;
    double last_theta = 0;

    // Count the angle accumulated going around the polygon. If
    // the sum is +2pi, it's CCW. Otherwise, we'll get -2pi.
    int sz = vec_length(poly);

    for (int i = 0; i <= sz; i++) {
        g2d_point_t p0, p1;
        p0 = poly->data[i % sz];
        p1 = poly->data[(i + 1) % sz];

        double this_theta = atan2(p1.y - p0.y, p1.x - p0.x);

        if (i > 0) {
            double dtheta = mod2pi(this_theta-last_theta);
            total_theta += dtheta;
        }

        last_theta = this_theta;
    }

    int ccw = (total_theta > 0);

    // reverse order if necessary.
    if (!ccw) {
        for (int i = 0; i < sz / 2; i++) {
            g2d_point_t a = poly->data[i];
            g2d_point_t b = poly->data[sz - 1 - i];

            poly->data[i] = b;
            poly->data[sz - 1 - i] = a;
        }
    }
}

int g2d_polygon_contains_point_ref(const g2d_poly_t *poly, const g2d_point_t *q)
{
    // use winding. If the point is inside the polygon, we'll wrap
    // around it (accumulating 6.28 radians). If we're outside the
    // polygon, we'll accumulate zero.
    int psz = vec_length(poly);

    double acc_theta = 0;

    double last_theta;

    for (int i = 0; i <= psz; i++) {
        g2d_point_t p = poly->data[i % psz];

        double this_theta = atan2(q->y - p.y, q->x - p.x);

        if (i != 0)
            acc_theta += mod2pi(this_theta - last_theta);

        last_theta = this_theta;
    }

    return acc_theta > M_PI;
}

/*
// sort by x coordinate, ascending
static int g2d_convex_hull_sort(const void *_a, const void *_b)
{
    double *a = (double*) _a;
    double *b = (double*) _b;

    if (a[0] < b[0])
        return -1;
    if (a[0] == b[0])
        return 0;
    return 1;
}
*/

/*
zarray_t *g2d_convex_hull2(const zarray_t *points)
{
    zarray_t *hull = zarray_copy(points);

    zarray_sort(hull, g2d_convex_hull_sort);

    int hsz = zarray_size(hull);
    int hout = 0;

    for (int hin = 1; hin < hsz; hin++) {
        double *p;
        zarray_get_volatile(hull, i, &p);

        // Everything to the right of hin is already convex. We now
        // add one point, p, which begins "connected" by two
        // (coincident) edges from the last right-most point to p.
        double *last;
        zarray_get_volatile(hull, hout, &last);

        // We now remove points from the convex hull by moving
    }

    return hull;
}
*/

// creates and returns a zarray(double[2]). The resulting polygon is
// CCW and implicitly closed. Unnecessary colinear points are omitted.
int g2d_convex_hull(const g2d_poly_t *points, g2d_poly_t *hull)
{
    vec_init(hull);

    // gift-wrap algorithm.

    // step 1: find left most point.
    vec_size_t insz = vec_length(points);

    // must have at least 2 points. (XXX need 3?)
    assert(insz >= 2);

    const g2d_point_t *pleft = NULL;
    for (int i = 0; i < insz; i++) {
        const g2d_point_t *p = &points->data[i];

        if (pleft == NULL || p->x < pleft->x)
            pleft = p;
    }

    // cannot be NULL since there must be at least one point.
    assert(pleft != NULL);

    vec_push(hull, *pleft);

    // step 2. gift wrap. Keep searching for points that make the
    // smallest-angle left-hand turn. This implementation is carefully
    // written to use only addition/subtraction/multiply. No division
    // or sqrts. This guarantees exact results for integer-coordinate
    // polygons (no rounding/precision problems).
    const g2d_point_t *p = pleft;

    while (1) {
        g2d_point_t *q = NULL;
        double n0 = 0, n1 = 0; // the normal to the line (p, q) (not
                       // necessarily unit length).

        // Search for the point q for which the line (p,q) is most "to
        // the right of" the other points. (i.e., every time we find a
        // point that is to the right of our current line, we change
        // lines.)
        for (int i = 0; i < insz; i++) {
            g2d_point_t *thisq = &points->data[i];

            if (thisq == p)
                continue;

            // the first time we find another point, we initialize our
            // value of q, forming the line (p,q)
            if (q == NULL) {
                q = thisq;
                n0 = q->y - p->y;
                n1 = -q->x + p->x;
            } else {
                // we already have a line (p,q). is point thisq RIGHT OF line (p, q)?
                double e0 = thisq->x - p->x, e1 = thisq->y - p->y;
                double dot = e0 * n0 + e1 * n1;

                if (dot > 0) {
                    // it is. change our line.
                    q = thisq;
                    n0 = q->y - p->y;
                    n1 = -q->x + p->x;
                }
            }
        }

        // we must have elected *some* line, so long as there are at
        // least 2 points in the polygon.
        assert(q != NULL);

        // loop completed?
        if (q == pleft)
            break;

        int colinear = 0;

        // is this new point colinear with the last two?
        if (vec_length(hull) > 1) {
            const g2d_point_t *o = &hull->data[vec_length(hull) - 2];

            double e0 = o->x - p->x;
            double e1 = o->y - p->y;

            if (n0*e0 + n1*e1 == 0)
                colinear = 1;
        }

        // if it is colinear, overwrite the last one.
        if (colinear) {
            hull->data[vec_length(hull) - 1] = *q;
        } else {
            vec_push(hull, *q);
        }
        p = q;
    }

    return 0;
}

// Find point p on the boundary of poly that is closest to q.
void g2d_polygon_closest_boundary_point(const g2d_poly_t *poly, const g2d_point_t *q, g2d_point_t *p)
{
    int psz = vec_length(poly);
    double min_dist = HUGE_VALF;

    for (int i = 0; i < psz; i++) {
        g2d_point_t p0, p1;
        p0 = poly->data[i];
        p1 = poly->data[(i + 1) % psz];

        g2d_line_segment_t seg;
        g2d_line_segment_init_from_points(&seg, &p0, &p1);

        g2d_point_t thisp;
        g2d_line_segment_closest_point(&seg, q, &thisp);

        double dist = g2d_distance(q, &thisp);
        if (dist < min_dist) {
            *p = thisp;
            min_dist = dist;
        }
    }
}

int g2d_polygon_contains_point(const g2d_poly_t *poly, const g2d_point_t *q)
{
    // use winding. If the point is inside the polygon, we'll wrap
    // around it (accumulating 6.28 radians). If we're outside the
    // polygon, we'll accumulate zero.
    int psz = vec_length(poly);
    assert(psz > 0);

    int last_quadrant;
    int quad_acc = 0;

    for (int i = 0; i <= psz; i++) {
        g2d_point_t p = poly->data[i % psz];

        // p[0] < q[0]       p[1] < q[1]    quadrant
        //     0                 0              0
        //     0                 1              3
        //     1                 0              1
        //     1                 1              2

        // p[1] < q[1]       p[0] < q[0]    quadrant
        //     0                 0              0
        //     0                 1              1
        //     1                 0              3
        //     1                 1              2

        int quadrant;
        if (p.x < q->x)
            quadrant = (p.y < q->y) ? 2 : 1;
        else
            quadrant = (p.y < q->y) ? 3 : 0;

        if (i > 0) {
            int dquadrant = quadrant - last_quadrant;

            // encourage a jump table by mapping to small positive integers.
            switch (dquadrant) {
                case -3:
                case 1:
                    quad_acc ++;
                    break;
                case -1:
                case 3:
                    quad_acc --;
                    break;
                case 0:
                    break;
                case -2:
                case 2:
                {
                    // get the previous point.
                    g2d_point_t p0 = poly->data[i - 1];

                    // Consider the points p0 and p (the points around the
                    //polygon that we are tracing) and the query point q.
                    //
                    // If we've moved diagonally across quadrants, we want
                    // to measure whether we have rotated +PI radians or
                    // -PI radians. We can test this by computing the dot
                    // product of vector (p0-q) with the vector
                    // perpendicular to vector (p-q)
                    double nx = p.y - q->y;
                    double ny = -p.x + q->x;

                    double dot = nx*(p0.x - q->x) + ny*(p0.y - q->y);
                    if (dot < 0)
                        quad_acc -= 2;
                    else
                        quad_acc += 2;

                    break;
                }
            }
        }

        last_quadrant = quadrant;
    }

    int v = (quad_acc >= 2) || (quad_acc <= -2);

    if (0 && v != g2d_polygon_contains_point_ref(poly, q)) {
        printf("FAILURE %d %d\n", v, quad_acc);
        exit(-1);
    }

    return v;
}

void g2d_line_init_from_points(g2d_line_t *line, const g2d_point_t *p0, const g2d_point_t *p1)
{
    line->p.x = p0->x;
    line->p.y = p0->y;
    line->u.x = p1->x - p0->x;
    line->u.y = p1->y - p0->y;

    // normalize u
    double mag = sqrtf(sq(line->u.x) + sq(line->u.y));
    line->u.x /= mag;
    line->u.y /= mag;
}

double g2d_line_get_coordinate(const g2d_line_t *line, const g2d_point_t *q)
{
    return (q->x - line->p.x) * line->u.x + (q->y - line->p.y) * line->u.y;
}

// Compute intersection of two line segments. If they intersect,
// result is stored in p and 1 is returned. Otherwise, zero is
// returned. p may be NULL.
int g2d_line_intersect_line(const g2d_line_t *linea, const g2d_line_t *lineb, g2d_point_t *p)
{
    // this implementation is many times faster than the original,
    // mostly due to avoiding a general-purpose LU decomposition in
    // Matrix.inverse().
    double m00, m01, m10, m11;
    double i00, i01;
    double b00, b10;

    m00 = linea->u.x;
    m01= -lineb->u.x;
    m10 = linea->u.y;
    m11= -lineb->u.y;

    // determinant of m
    double det = m00*m11-m01*m10;

    // parallel lines?
    if (fabs(det) < 0.00000001)
        return 0;

    // inverse of m
    i00 = m11/det;
    i01 = -m01/det;

    b00 = lineb->p.x - linea->p.x;
    b10 = lineb->p.y - linea->p.y;

    double x00; //, x10;
    x00 = i00*b00+i01*b10;

    if (p != NULL) {
        p->x = linea->u.x * x00 + linea->p.x;
        p->y = linea->u.y * x00 + linea->p.y;
    }

    return 1;
}


void g2d_line_segment_init_from_points(g2d_line_segment_t *seg, const g2d_point_t *p0, const g2d_point_t *p1)
{
    g2d_line_init_from_points(&seg->line, p0, p1);
    seg->p1.x = p1->x;
    seg->p1.y = p1->y;
}

// Find the point p on segment seg that is closest to point q.
void g2d_line_segment_closest_point(const g2d_line_segment_t *seg, const g2d_point_t *q, g2d_point_t *p)
{
    double a = g2d_line_get_coordinate(&seg->line, &seg->line.p);
    double b = g2d_line_get_coordinate(&seg->line, &seg->p1);
    double c = g2d_line_get_coordinate(&seg->line, q);

    if (a < b)
        c = dclamp(c, a, b);
    else
        c = dclamp(c, b, a);

    p->x = seg->line.p.x + c * seg->line.u.x;
    p->y = seg->line.p.y + c * seg->line.u.y;
}

// Compute intersection of two line segments. If they intersect,
// result is stored in p and 1 is returned. Otherwise, zero is
// returned. p may be NULL.
int g2d_line_segment_intersect_segment(const g2d_line_segment_t *sega, const g2d_line_segment_t *segb, g2d_point_t *p)
{
    g2d_point_t tmp;

    if (!g2d_line_intersect_line(&sega->line, &segb->line, &tmp))
        return 0;

    double a = g2d_line_get_coordinate(&sega->line, &sega->line.p);
    double b = g2d_line_get_coordinate(&sega->line, &sega->p1);
    double c = g2d_line_get_coordinate(&sega->line, &tmp);

    // does intersection lie on the first line?
    if ((c<a && c<b) || (c>a && c>b))
        return 0;

    a = g2d_line_get_coordinate(&segb->line, &segb->line.p);
    b = g2d_line_get_coordinate(&segb->line, &segb->p1);
    c = g2d_line_get_coordinate(&segb->line, &tmp);

    // does intersection lie on second line?
    if ((c<a && c<b) || (c>a && c>b))
        return 0;

    if (p != NULL) {
        *p = tmp;
    }

    return 1;
}

// Compute intersection of a line segment and a line. If they
// intersect, result is stored in p and 1 is returned. Otherwise, zero
// is returned. p may be NULL.
int g2d_line_segment_intersect_line(const g2d_line_segment_t *seg, const g2d_line_t *line, g2d_point_t *p)
{
    g2d_point_t tmp;

    if (!g2d_line_intersect_line(&seg->line, line, &tmp))
        return 0;

    double a = g2d_line_get_coordinate(&seg->line, &seg->line.p);
    double b = g2d_line_get_coordinate(&seg->line, &seg->p1);
    double c = g2d_line_get_coordinate(&seg->line, &tmp);

    // does intersection lie on the first line?
    if ((c<a && c<b) || (c>a && c>b))
        return 0;

    if (p != NULL) {
        *p = tmp;
    }

    return 1;
}

// do the edges of polya and polyb collide? (Does NOT test for containment).
int g2d_polygon_intersects_polygon(const g2d_poly_t *polya, const g2d_poly_t *polyb)
{
    // do any of the line segments collide? If so, the answer is no.

    // dumb N^2 method.
    for (vec_size_t ia = 0, sza = vec_length(polya); ia < sza; ia++) {
        g2d_point_t pa0 = polya->data[ia];
        g2d_point_t pa1 = polya->data[(ia + 1) % sza];

        g2d_line_segment_t sega;
        g2d_line_segment_init_from_points(&sega, &pa0, &pa1);

        for (vec_size_t ib = 0, szb = vec_length(polyb); ib < szb; ib++) {
            g2d_point_t pb0 = polyb->data[ib];
            g2d_point_t pb1 = polyb->data[(ib + 1) % szb];

            g2d_line_segment_t segb;
            g2d_line_segment_init_from_points(&segb, &pb0, &pb1);

            if (g2d_line_segment_intersect_segment(&sega, &segb, NULL))
                return 1;
        }
    }

    return 0;
}

// does polya completely contain polyb?
int g2d_polygon_contains_polygon(const g2d_poly_t *polya, const g2d_poly_t *polyb)
{
    // do any of the line segments collide? If so, the answer is no.
    if (g2d_polygon_intersects_polygon(polya, polyb))
        return 0;

    // if none of the edges cross, then the polygon is either fully
    // contained or fully outside.
    g2d_point_t p = polyb->data[0];
    return g2d_polygon_contains_point(polya, &p);
}

// compute a point that is inside the polygon. (It may not be *far* inside though)
void g2d_polygon_get_interior_point(const g2d_poly_t *poly, g2d_point_t *p)
{
    // take the first three points, which form a triangle. Find the middle point
    g2d_point_t a = poly->data[0],
                b = poly->data[1],
                c = poly->data[2];

    p->x = (a.x + b.x + c.x) / 3;
    p->y = (a.y + b.y + c.y) / 3;
}

int g2d_polygon_overlaps_polygon(const g2d_poly_t *polya, const g2d_poly_t *polyb)
{
    // do any of the line segments collide? If so, the answer is yes.
    if (g2d_polygon_intersects_polygon(polya, polyb))
        return 1;

    // if none of the edges cross, then the polygon is either fully
    // contained or fully outside.
    g2d_point_t p;
    g2d_polygon_get_interior_point(polyb, &p);

    if (g2d_polygon_contains_point(polya, &p))
        return 1;

    g2d_polygon_get_interior_point(polya, &p);

    if (g2d_polygon_contains_point(polyb, &p))
        return 1;

    return 0;
}

static int double_sort_up(const void *_a, const void *_b)
{
    double a = *((double*) _a);
    double b = *((double*) _b);

    if (a < b)
        return -1;

    if (a == b)
        return 0;

    return 1;
}

// Compute the crossings of the polygon along line y, storing them in
// the array x. X must be allocated to be at least as long as
// zarray_size(poly). X will be sorted, ready for
// rasterization. Returns the number of intersections (and elements
// written to x).
/*
  To rasterize, do something like this:

  double res = 0.099;
  for (double y = y0; y < y1; y += res) {
  double xs[zarray_size(poly)];

  int xsz = g2d_polygon_rasterize(poly, y, xs);
  int xpos = 0;
  int inout = 0; // start off "out"

  for (double x = x0; x < x1; x += res) {
      while (x > xs[xpos] && xpos < xsz) {
        xpos++;
        inout ^= 1;
      }

    if (inout)
       printf("y");
    else
       printf(" ");
  }
  printf("\n");
*/

// returns the number of x intercepts
int g2d_polygon_rasterize(const g2d_poly_t *poly, double y, double *x)
{
    vec_size_t sz = vec_length(poly);

    g2d_line_t line;
    if (1) {
        g2d_point_t p0 = { 0, y };
        g2d_point_t p1 = { 1, y };

        g2d_line_init_from_points(&line, &p0, &p1);
    }

    int xpos = 0;

    for (int i = 0; i < sz; i++) {
        g2d_line_segment_t seg;
        g2d_point_t p0 = poly->data[i];
        g2d_point_t p1 = poly->data[(i + 1) % sz];

        g2d_line_segment_init_from_points(&seg, &p0, &p1);

        g2d_point_t q;
        if (g2d_line_segment_intersect_line(&seg, &line, &q))
            x[xpos++] = q.x;
    }

    qsort(x, xpos, sizeof(double), double_sort_up);

    return xpos;
}
