#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "quaternion.h"

void estimate_yaw_pitch_dist(int im_w, int im_h, int left, int top, int right, int bot,
                            struct euler_t *eul, struct quaternion_t *quat, float *distance);
#endif
