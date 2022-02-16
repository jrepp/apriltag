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

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>
#include <math.h>

#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"

// This demo program is also used to profile and test the internals of the apriltag library, the internals should
// under normal usage not be used.
#include "common/getopt.h"
#include "common/image_u8.h"
#include "common/image_u8x4.h"
#include "common/pjpeg.h"
#include "common/vec.h"
#include "common/apriltag_detector.h"

typedef struct {
    vec_define_fields(image_u8_t*)
} vec_image_t;

// Invoke:
//
// tagtest [options] input.pnm

int main(int argc, char *argv[])
{
    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, 'i', "iters", "1", "Repeat processing on input set this many times");
    getopt_add_int(getopt, 't', "threads", "1", "Use this many CPU threads");
    getopt_add_int(getopt, 'a', "hamming", "1", "Detect tags with up to this many bit errors.");
    getopt_add_double(getopt, 'x', "decimate", "2.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input; negative sharpens");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");

    if (!getopt_parse(getopt, argc, argv, 1) || getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options] <input files>\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    const vec_str_t *inputs = getopt_get_extra_args(getopt);

    apriltag_family_t *tf = NULL;
    const char *famname = getopt_get_string(getopt, "family");
    if (!strcmp(famname, "tag36h11")) {
        tf = tag36h11_create();
    } else if (!strcmp(famname, "tag25h9")) {
        tf = tag25h9_create();
    } else if (!strcmp(famname, "tag16h5")) {
        tf = tag16h5_create();
    } else if (!strcmp(famname, "tagCircle21h7")) {
        tf = tagCircle21h7_create();
    } else if (!strcmp(famname, "tagCircle49h12")) {
        tf = tagCircle49h12_create();
    } else if (!strcmp(famname, "tagStandard41h12")) {
        tf = tagStandard41h12_create();
    } else if (!strcmp(famname, "tagStandard52h13")) {
        tf = tagStandard52h13_create();
    } else if (!strcmp(famname, "tagCustom48h12")) {
        tf = tagCustom48h12_create();
    } else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }

    apriltag_detector_config_t config;
    apriltag_detector_config_default(&config);

    // Detector parameters
    config.quad_decimate = getopt_get_double(getopt, "decimate");
    config.quad_sigma = getopt_get_double(getopt, "blur");
    config.nthreads = getopt_get_int(getopt, "threads");
    config.debug = getopt_get_bool(getopt, "debug");
    config.refine_edges = getopt_get_bool(getopt, "refine-edges");

    apriltag_detector_t *td = apriltag_detector_create(&config);
    apriltag_detector_add_family_bits(td, tf, getopt_get_int(getopt, "hamming"));

    // Demo program parameters
    int quiet = getopt_get_bool(getopt, "quiet");
    int maxiters = getopt_get_int(getopt, "iters");

    const int hamm_hist_max = 10;

    // Load the source images
    vec_image_t images;
    vec_init(&images);
    for (int input = 0; input < vec_length(inputs); input++) {
        char *path = vec_get(inputs, input);
        if (!quiet)
            printf("loading %s\n", path);
        else
            printf("%20s ", path);

        image_u8_t *im = NULL;
        if (str_ends_with(path, "pnm") || str_ends_with(path, "PNM") ||
            str_ends_with(path, "pgm") || str_ends_with(path, "PGM"))
            im = image_u8_create_from_pnm(path);
        else if (str_ends_with(path, "jpg") || str_ends_with(path, "JPG")) {
            int err = 0;
            pjpeg_t *pjpeg = pjpeg_create_from_file(path, 0, &err);
            if (pjpeg == NULL) {
                printf("pjpeg error %d\n", err);
                continue;
            }

            if (1) {
                im = pjpeg_to_u8_baseline(pjpeg);
            } else {
                printf("illumination invariant\n");

                image_u8x3_t *imc = pjpeg_to_u8x3_baseline(pjpeg);

                im = image_u8_create(imc->width, imc->height);

                for (int y = 0; y < imc->height; y++) {
                    for (int x = 0; x < imc->width; x++) {
                        double r = imc->buf[y * imc->stride + 3 * x + 0] / 255.0;
                        double g = imc->buf[y * imc->stride + 3 * x + 1] / 255.0;
                        double b = imc->buf[y * imc->stride + 3 * x + 2] / 255.0;

                        double alpha = 0.42;
                        double v = 0.5 + log(g) - alpha * log(b) - (1 - alpha) * log(r);
                        int iv = v * 255;
                        if (iv < 0)
                            iv = 0;
                        if (iv > 255)
                            iv = 255;

                        im->buf[y * im->stride + x] = iv;
                    }
                }
                image_u8x3_destroy(imc);
                if (td->config.debug)
                    image_u8_write_pnm(im, "debug_invariant.pnm");
            }

            pjpeg_destroy(pjpeg);
        }

        if (im == NULL) {
            printf("couldn't load %s\n", path);
            continue;
        }
        vec_push(&images, im);
    }

    for (int iter = 0; iter < maxiters; iter++) {

        int total_quads = 0;
        int total_hamm_hist[hamm_hist_max];
        memset(total_hamm_hist, 0, sizeof(total_hamm_hist));
        double total_time = 0;

        if (maxiters > 1)
            printf("iter %d / %d\n", iter + 1, maxiters);

        vec_size_t i;
        image_u8_t *im;
        vec_foreach(&images, im, i) {

            int hamm_hist[hamm_hist_max];
            memset(hamm_hist, 0, sizeof(hamm_hist));

            apriltag_detection_t *det = apriltag_detector_detect(td, im);
            for (int di = 0; det != NULL; det = apriltag_detector_next_detection(td), ++di) {
                if (!quiet) {
                    printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
                           di, det->family->nbits, det->family->h, det->id, det->hamming, det->decision_margin);
                }
                hamm_hist[det->hamming]++;
                total_hamm_hist[det->hamming]++;
            }

            if (!quiet) {
                timeprofile_display(td->tp);
            }

            total_quads += td->nquads;
            double t = timeprofile_total_utime(td->tp) / 1.0E3;
            total_time += t;

            if (!quiet) {
                printf("hamm ");

                for (int hi = 0; hi < hamm_hist_max; hi++)
                    printf("%5d ", hamm_hist[hi]);

                printf("time %12.3f ", t);
                printf("quads %5d", td->nquads);

                printf("\n");
            }
        }

        printf("Summary\n");

        printf("hamm ");

        for (int hi = 0; hi < hamm_hist_max; hi++)
            printf("%5d ", total_hamm_hist[hi]);
        printf("time %12.3f ", total_time);
        printf("quads %5d", total_quads);
        printf("\n");
    }

    vec_each(&images, image_u8_destroy);
    vec_deinit(&images);

    // don't deallocate contents of inputs; those are the argv
    apriltag_detector_destroy(td);

    if (!strcmp(famname, "tag36h11")) {
        tag36h11_destroy(tf);
    } else if (!strcmp(famname, "tag25h9")) {
        tag25h9_destroy(tf);
    } else if (!strcmp(famname, "tag16h5")) {
        tag16h5_destroy(tf);
    } else if (!strcmp(famname, "tagCircle21h7")) {
        tagCircle21h7_destroy(tf);
    } else if (!strcmp(famname, "tagCircle49h12")) {
        tagCircle49h12_destroy(tf);
    } else if (!strcmp(famname, "tagStandard41h12")) {
        tagStandard41h12_destroy(tf);
    } else if (!strcmp(famname, "tagStandard52h13")) {
        tagStandard52h13_destroy(tf);
    } else if (!strcmp(famname, "tagCustom48h12")) {
        tagCustom48h12_destroy(tf);
    }

    getopt_destroy(getopt);

    return 0;
}
