#ifndef INCLUDED_APRILTAG_DETECTOR_H
#define INCLUDED_APRILTAG_DETECTOR_H

#include "apriltag.h"
#include "common/vec.h"
#include "common/timeprofile.h"
#include "common/matd.h"
#include "common/workerpool.h"

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

typedef struct point {
    double x, y;
} point_t;

typedef struct {
    vec_define_fields(point_t)
} vec_point_t;

typedef struct quad
{
    float p[4][2]; // corners

    bool reversed_border;

    // H: tag coordinates ([-1,1] at the black corners) to pixels
    // Hinv: pixels to tag
    matd_t *H, *Hinv;
} atquad_t;

typedef struct {
    vec_define_fields(atquad_t)
} vec_atquad_t;

typedef struct {
    vec_define_fields(apriltag_detection_t)
} vec_apriltag_detection_t;

typedef struct {
    vec_define_fields(apriltag_family_t*)
} vec_apriltag_family_ptr_t;

struct apriltag_detector
{
    ///////////////////////////////////////////////////////////////
    // External configuration parameters
    struct apriltag_detector_config config;

    ///////////////////////////////////////////////////////////////
    // Detector memory
    vec_apriltag_detection_t detections;
    vec_atquad_t quads;

    // Current detection cursor
    vec_size_t detection_index;

    // Cache the quad image pre-process data between detections
    image_u8_t *im_quads;

    ///////////////////////////////////////////////////////////////
    // Buffer data

    ///////////////////////////////////////////////////////////////
    // Statistics relating to last processed frame
    timeprofile_t *tp;

    uint32_t nedges;
    uint32_t nsegments;
    uint32_t nquads;

    // Not freed on apriltag_destroy; a tag family can be shared
    // between multiple users. The user should ultimately destroy the
    // tag family passed into the constructor.
    vec_apriltag_family_ptr_t tag_families;

    // Used to manage multi-threading.
    workerpool_t *wp;

    // Used for thread safety.
    pthread_mutex_t mutex;
};

#endif // INCLUDED_APRILTAG_DETECTOR_H
