#ifdef __cplusplus
extern "C" {
#endif

#ifndef QUATERNION_H
#define QUATERNION_H

struct quaternion_t
{
    double w;
    double x;
    double y;
    double z;
};

struct euler_t
{
    double x;
    double y;
    double z;
};

double quat_norm(struct quaternion_t *q);
void quat2eul(struct quaternion_t *q, struct euler_t *e);
void eul2quat(struct quaternion_t *q, struct euler_t *e);
void quat_prod(struct quaternion_t *q, struct quaternion_t *q1, struct quaternion_t *q2);
void quat_mul(struct quaternion_t *q, struct quaternion_t *q1, struct quaternion_t *q2);
void quat_add(struct quaternion_t *q, struct quaternion_t *q1, struct quaternion_t *q2);
void quat_conj(struct quaternion_t *q);
void quat_scale(struct quaternion_t *q, double d);
void eul_conj(struct euler_t *e);
double quat_msqe(struct quaternion_t *q1, struct quaternion_t *q2);
double eul_msqe(struct euler_t *e1, struct euler_t *e2);

#endif

#ifdef __cplusplus
}
#endif
