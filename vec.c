#include "vec.h"

void float3_add_mutate(struct float3 *a, const struct float3 *b)
{
    a->x += b->x;
    a->y += b->y;
    a->z += b->z;
}

void float3_sub_mutate(struct float3 *a, const struct float3 *b)
{
    a->x -= b->x;
    a->y -= b->y;
    a->z -= b->z;
}

float float3_dot_prod(const struct float3 *a, const struct float3 *b)
{
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

float float3_len(const struct float3 *v)
{
    return sqrtf(float3_dot_prod(v, v));
}

void float3_mult_mutate(struct float3 *v, float c)
{
    v->x *= c;
    v->y *= c;
    v->z *= c;
}

void float3_norm_mutate(struct float3 *v)
{
    float3_mult_mutate(v, 1 / float3_len(v));   
}

void float3_cpy(struct float3 *a, const struct float3 *b)
{
    a->x = b->x;
    a->y = b->y;
    a->z = b->z;
}