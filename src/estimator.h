#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "quaternion.h"

#define MICROSOFT_FOVH 61
#define PHANTOM3_FOVH 81

void estimate_yaw_pitch_dist(int im_w, int im_h, int left, int top, int right, int bot,
                            struct euler_t *eul, struct quaternion_t *quat, float *distance);
#endif
