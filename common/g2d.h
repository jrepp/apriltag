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

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "common/vec.h"
#include "common/math_util.h"

// This library tries to avoid needless proliferation of types.
//
// A point is a double[2]. (Note that when passing a double[2] as an
// argument, it is passed by pointer, not by value.)
//
// A polygon is a g2d_poly_t array of g2d_point_t. (Note that in this case, the
// array contains the actual vertex data, and not merely a pointer to
// some other data. IMPORTANT: A polygon must be specified in CCW
// order.  It is implicitly closed (do not list the same point at the
// beginning at the end.
//
// Where sensible, it is assumed that objects should be allocated
// sparingly; consequently "init" style methods, rather than "create"
// methods are used.

////////////////////////////////////////////////////////////////////
// 2D points
typedef struct
{
    double x, y;
} g2d_point_t;

inline static double g2d_distance(const g2d_point_t *a, const g2d_point_t *b)
{
    return sqrtf(sq(a->x - b->y) + sq(a->y - b->y));
}

////////////////////////////////////////////////////////////////////
// Lines
typedef struct
{
    // Internal representation: a point that the line goes through (p) and
    // the direction of the line (u).
    g2d_point_t p;
    g2d_point_t u; // always a unit vector
} g2d_line_t;

// initialize a line object.
void g2d_line_init_from_points(g2d_line_t *line, const g2d_point_t *p0, const g2d_point_t *p1);

// The line defines a one-dimensional coordinate system whose origin
// is p. Where is q? (If q is not on the line, the point nearest q is
// returned.
double g2d_line_get_coordinate(const g2d_line_t *line, const g2d_point_t *q);

// Intersect two lines. The intersection, if it exists, is written to
// p (if not NULL), and 1 is returned. Else, zero is returned.
int g2d_line_intersect_line(const g2d_line_t *linea, const g2d_line_t *lineb, g2d_point_t *p);

////////////////////////////////////////////////////////////////////
// Line Segments. line.p is always one endpoint; p1 is the other
// endpoint.
typedef struct
{
    g2d_line_t line;
    g2d_point_t p1;
} g2d_line_segment_t;


typedef struct
{
    vec_define_fields(g2d_point_t)
} g2d_poly_t;

void g2d_line_segment_init_from_points(g2d_line_segment_t *seg, const g2d_point_t *p0, const g2d_point_t *p1);

// Intersect two segments. The intersection, if it exists, is written
// to p (if not NULL), and 1 is returned. Else, zero is returned.
int g2d_line_segment_intersect_segment(const g2d_line_segment_t *sega, const g2d_line_segment_t *segb, g2d_point_t *p);

void g2d_line_segment_closest_point(const g2d_line_segment_t *seg, const g2d_point_t *q, g2d_point_t *p);
double g2d_line_segment_closest_point_distance(const g2d_line_segment_t *seg, const g2d_point_t *q);

////////////////////////////////////////////////////////////////////
// Polygons

int g2d_polygon_create_data(const g2d_point_t *points, int sz, g2d_poly_t *poly);

int g2d_polygon_create_zeros(int sz, g2d_poly_t *poly);

int g2d_polygon_create_empty(g2d_poly_t *v);

void g2d_polygon_add(g2d_poly_t *poly, const g2d_point_t *p);

// Takes a polygon in either CW or CCW and modifies it (if necessary)
// to be CCW.
void g2d_polygon_make_ccw(g2d_poly_t *poly);

// Generate a convex hull from points
int g2d_convex_hull(const g2d_poly_t *points, g2d_poly_t *hull);

// Find point p on the boundary of poly that is closest to q.
void g2d_polygon_closest_boundary_point(const g2d_poly_t *poly, const g2d_point_t *q, g2d_point_t *p);

// Return 1 if point q lies within poly.
int g2d_polygon_contains_point(const g2d_poly_t *poly, const g2d_point_t *q);
int g2d_polygon_contains_point_ref(const g2d_poly_t *poly, const g2d_point_t *q);

// Do the edges of the polygons cross? (Does not test for containment).
int g2d_polygon_intersects_polygon(const g2d_poly_t *polya, const g2d_poly_t *polyb);

// Does polya completely contain polyb?
int g2d_polygon_contains_polygon(const g2d_poly_t *polya, const g2d_poly_t *polyb);

// Is there some point which is in both polya and polyb?
int g2d_polygon_overlaps_polygon(const g2d_poly_t *polya, const g2d_poly_t *polyb);

// returns the number of points written to x. see comments.
int g2d_polygon_rasterize(const g2d_poly_t *poly, double y, double *x);

#ifdef __cplusplus
}
#endif
