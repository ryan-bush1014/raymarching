#include <math.h>

struct float3
{
    float x;
    float y;
    float z;
};

void float3_add_mutate(struct float3 *a, const struct float3 *b);

void float3_sub_mutate(struct float3 *a, const struct float3 *b);

float float3_dot_prod(const struct float3 *a, const struct float3 *b);

float float3_len(const struct float3 *v);

void float3_mult_mutate(struct float3 *v, float c);

void float3_norm_mutate(struct float3 *v);

void float3_cpy(struct float3 *a, const struct float3 *b);

void float3_lambert(const struct float3 *unit_sphere_pos, float *x, float *y);
