#include <math.h>
#include "quaternion.h"

double quat_norm(struct quaternion_t *q)
{
    double d = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);

    q->w /= d;
    q->x /= d;
    q->y /= d;
    q->z /= d;

    return d;
}

// degrees in radian
void quat2eul(struct quaternion_t *q, struct euler_t *e)
{
    quat_norm(q);

    e->z = atan2(2 * (q->x * q->y + q->w * q->z), q->w * q->w + q->x * q->x - q->y * q->y - q->z * q->z);
    e->y = asin(-2 * (q->x * q->z - q->w * q->y));
    e->x = atan2(2 * (q->y * q->z + q->w * q->x), q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z);
}

// degrees in radian
void eul2quat(struct quaternion_t *q, struct euler_t *e)
{
    double cx = cos(e->x / 2);
    double cy = cos(e->y / 2);
    double cz = cos(e->z / 2);

    double sx = sin(e->x / 2);
    double sy = sin(e->y / 2);
    double sz = sin(e->z / 2);

    q->w = cz * cy * cx + sz * sy * sx;
    q->x = cz * cy * sx - sz * sy * cx;
    q->y = cz * sy * cx + sz * cy * sx;
    q->z = sz * cy * cx - cz * sy * sx;
}

void quat_prod(struct quaternion_t *q, struct quaternion_t *q1, struct quaternion_t *q2)
{
    q->w = q1->w * q2->w;
    q->x = q1->x * q2->x;
    q->y = q1->y * q2->y;
    q->z = q1->z * q2->z;
}

void quat_mul(struct quaternion_t *q, struct quaternion_t *q1, struct quaternion_t *q2)
{
    q->w = -q1->x * q2->x - q1->y * q2->y - q1->z * q2->z + q1->w * q2->w;
    q->x =  q1->x * q2->w + q1->y * q2->z - q1->z * q2->y + q1->w * q2->x;
    q->y = -q1->x * q2->z + q1->y * q2->w + q1->z * q2->x + q1->w * q2->y;
    q->z =  q1->x * q2->y - q1->y * q2->x + q1->z * q2->w + q1->w * q2->z;
}

void quat_add(struct quaternion_t *q, struct quaternion_t *q1, struct quaternion_t *q2)
{
    q->w = q1->w + q2->w;
    q->x = q1->x + q2->x;
    q->y = q1->y + q2->y;
    q->z = q1->z + q2->z;
}

void quat_conj(struct quaternion_t *q)
{
    q->x = -q->x;
    q->y = -q->y;
    q->z = -q->z;
}

void quat_scale(struct quaternion_t *q, double d)
{
    q->w *= d;
    q->x *= d;
    q->y *= d;
    q->z *= d;
}

void eul_conj(struct euler_t *e)
{
    if (e->x < 0) e->x += M_PI;
    else e->x -= M_PI;

    if (e->y < 0) e->y += M_PI;
    else e->y -= M_PI;

    if (e->z < 0) e->z += M_PI;
    else e->z -= M_PI;

    e->y = -e->y;
}

double quat_msqe(struct quaternion_t *q1, struct quaternion_t *q2)
{
    return (pow(q1->w - q2->w, 2) +
            pow(q1->x - q2->x, 2) +
            pow(q1->y - q2->y, 2) +
            pow(q1->z - q2->z, 2)) / 4;
}

double eul_msqe(struct euler_t *e1, struct euler_t *e2)
{
    return (pow(e1->x - e2->x, 2) +
            pow(e1->y - e2->y, 2) +
            pow(e1->z - e2->z, 2)) / 3;
}
