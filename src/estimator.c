#include <math.h>
#include "estimator.h"
#include "quaternion.h"

/* Angles measure from center image
    + pitch angle when target is closer to top
    + yaw angle when target is right from center
    target yaw and pitch values
*/
void estimate_yaw_pitch_dist(int im_w, int im_h, int left, int top, int right, int bot,
                            struct euler_t *eul, struct quaternion_t *quat, float *distance)
{
    // TODO get these from configuration

    const float CameraFOVH_rad = 81 * M_PI / 180;
    const float targetsize = 0.2; // meter

    double CameraFOVV_rad = CameraFOVH_rad * im_h / im_w;
    // multiplier based on constants and camera settings
    const float m = im_w * targetsize * 2 * tan(CameraFOVH_rad / 2);

    //int h = bot - top;
    float w = (float)(right - left);

/*
    double pixangle = CameraFOVH_rad / im_w;
    *distance = targetsize / tan(w * pixangle);
*/


    *distance =  m / w;

    double x_offs2x = (double)(left + right - im_w);
    double y_offs2x = (double)(top + bot - im_h);

    /*
    alpha = cam_fovh
    w = frame pixel width
    d = distance camera and center of plane
    b = horiz pixel distance from center
    beta = angle of target from center point

    tan(alpha/2) = w/2/d => d = w / (2 * tan(alpha/2))
    tan(beta) = b / d
    beta = atan(2 * b * tan(alpha/2) / w)
    */

    // standard aeronautic "NED" "left hand"
    // x axis points out - roll
    // y axis points right (towards right of screen) - pitch
    // z axis points down  (towards bottom of screen/view = camera 'tilt' down) - yaw
    // screen y=0, x=0 is top left of screen

    eul->z = atan2(x_offs2x * tan(CameraFOVH_rad / 2), im_w); // yaw
    eul->y = atan2(y_offs2x * tan(CameraFOVV_rad / 2), im_h); // pitch
    eul->x = 0; // camera roll

    eul2quat(quat, eul);
}
