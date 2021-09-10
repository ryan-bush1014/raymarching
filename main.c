// A software raymarching engine that can render any geometry represented as an SDF

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "lodepng.h"
#include "vec.h"

#define WIDTH 1920.0f
#define HEIGHT 1080.0f
#define TOUCH_DIST 0.001f
#define MAX_IT 500
#define MAX_DIST 60.0f
#define SMOOTHNESS 0.1f
#define HALFPI 1.57079632679
#define PI 3.14159265359
#define TAU 6.28318530718
#define STAGE_LEN 1

const float SQR_DIST = MAX_DIST * MAX_DIST;

struct float3 backgound_col = {0, 0, 0};

struct float3 cam_pos = {0.0f, 4.0f, 0.0f};
struct float3 light_position = {-3.0f, 4.0f, -5.0f};

struct stage_object
{
    // Signed distance function for each stage object
    float (*sdf)(const struct float3 *pos, const struct float3 *coords_a, const struct float3 *coords_b, float size);

    // To allow for different types of geometry, two coordinates and a size are allocated for use by the SDF
    // NOTE: Some simple shapes (e.g. sphere) don't require both of these coordinates.
    struct float3 *coords_a;
    struct float3 *coords_b;
    float size;

    // The RGB value of the surface
    struct float3 *color;
};


// Create an array of stage objects
struct stage_object stage[STAGE_LEN] = {0};


// float3_rotate_x(v, theta) rotate the vector v around the x-axis by the angle theta
// requires: v is not NULL
// effects: mutates *v
void float3_rotate_x(struct float3 *v, float theta)
{
    assert(v);
    float csn = cosf(theta);
    float sn = sinf(theta);

    // create rows of rotational matrix
    struct float3 r1 = {1.0f, 0.0f, 0.0f};
    struct float3 r2 = {0.0f, csn, -sn};
    struct float3 r3 = {0.0f, sn, csn};

    // manually multiply vector by matrix
    float new_x = float3_dot_prod(v, &r1);
    float new_y = float3_dot_prod(v, &r2);
    float new_z = float3_dot_prod(v, &r3);
    v->x = new_x;
    v->y = new_y;
    v->z = new_z;
}

// float3_rotate_y(v, theta) rotate the vector v around the y-axis by the angle theta
// requires: v is not NULL
// effects: mutates *v
void float3_rotate_y(struct float3 *v, float theta)
{
    assert(v);
    float csn = cosf(theta);
    float sn = sinf(theta);

    // create rows of rotational matrix
    struct float3 r1 = {csn, 0.0f, sn};
    struct float3 r2 = {0.0f, 1.0f, 0.0f};
    struct float3 r3 = {-sn, 0.0f, csn};

    // manually multiply vector by matrix
    float new_x = float3_dot_prod(v, &r1);
    float new_y = float3_dot_prod(v, &r2);
    float new_z = float3_dot_prod(v, &r3);
    v->x = new_x;
    v->y = new_y;
    v->z = new_z;
}

// float3_rotate_z(v, theta) rotate the vector v around the z-axis by the angle theta
// requires: v is not NULL
// effects: mutates *v
void float3_rotate_z(struct float3 *v, float theta)
{
    assert(v);
    float csn = cosf(theta);
    float sn = sinf(theta);

    // create rows of rotational matrix
    struct float3 r1 = {csn, -sn, 0.0f};
    struct float3 r2 = {sn, csn, 0.0f};
    struct float3 r3 = {0.0f, 0.0f, 1.0f};

    // manually multiply vector by matrix
    float new_x = float3_dot_prod(v, &r1);
    float new_y = float3_dot_prod(v, &r2);
    float new_z = float3_dot_prod(v, &r3);
    v->x = new_x;
    v->y = new_y;
    v->z = new_z;
}

// nfmod(a, b) determines the modulus of a over b
// requires: b != 0
float nfmod(float a, float b)
{
    assert(b != 0);
    return a - b * floor(a / b);
}

// fclampf(n, a, b) returns n within the range [a, b] or the closest endpoint if not within range
// requires: a <= b
float fclampf(float n, float a, float b)
{
    assert(a <= b);
    return fmaxf(a, fminf(n, b));
}

// lerp(a, b, f) linearly interpolates between a and b with a proportion of p
float lerp(float a, float b, float p)
{
    return (a * (1.0 - p)) + (b * p);
}

// smin(a, b, k, a_col b_col, out_col) computes the 'smooth min' of a and b, and linearly interpolates a_col and b_col correspondingly
// requires: a_col, b_col, and out_col are not NULL
// effects: modifies *out_col
float smin(float a, float b, float k, struct float3 *a_col, struct float3 *b_col, struct float3 *out_col)
{
    float h = fclampf(0.5f + 0.5f * (b - a) / k, 0.0f, 1.0f);
    out_col->x = lerp(b_col->x, a_col->x, h);
    out_col->y = lerp(b_col->y, a_col->y, h);
    out_col->z = lerp(b_col->z, a_col->z, h);
    return lerp(b, a, h) - k * h * (1.0f - h);
}

///////////////////////////////////////////////////////////////////////
//                                                                   //
//            The following are signed distance functions            //
//   They take in a point and output the distance to a given shape   //
// They are standardized, so they may not make use of all parameters //
//                                                                   //
///////////////////////////////////////////////////////////////////////

float dist_to_cube(const struct float3 *pos, const struct float3 *coords_a, const struct float3 *coords_b, float size)
{
    struct float3 z = {0};
    float3_cpy(&z, pos);
    z.x = fabs(z.x - coords_a->x) - size;
    z.y = fabs(z.y - coords_a->y) - size;
    z.z = fabs(z.z - coords_a->z) - size;
    float d = z.x;
    d = fmax(d, z.y);
    d = fmax(d, z.z);
    return d;
}

float dist_to_circ(const struct float3 *pos, const struct float3 *coords_a, const struct float3 *coords_b, float size)
{
    struct float3 z = {0};
    float3_cpy(&z, pos);
    float3_sub_mutate(&z, coords_a);
    return float3_len(&z) - size;
}

// dist_to_scene(pos, color) determines the distance from pos to the nearest scene geometry
//   and mutates *color to the color of that geometry
// requires: pos and color are not NULL
// effects: Modifies *color
float dist_to_scene(const struct float3 *pos, struct float3 *color)
{
    assert(pos);
    assert(color);
    if (!STAGE_LEN)
        return MAX_DIST + 1;

    float d = (stage->sdf)(pos, stage->coords_a, stage->coords_b, stage->size);
    float3_cpy(color, stage->color);
    for (int i = 1; i < STAGE_LEN; ++i)
    {
        struct float3 out_col = {0};
        float b = ((stage + i)->sdf)(pos, (stage + i)->coords_a, (stage + i)->coords_b, (stage + i)->size);
        d = smin(d, b, SMOOTHNESS, color, (stage + i)->color, &out_col);
        float3_cpy(color, &out_col);
    }
    return d;
}

// position_to_normal(p, f) computes the surface normal at position p given the sdf f
// requires: p and f are not NULL
// effects: Modifies *p
void position_to_normal(struct float3 *p, float (*f)(const struct float3 *, struct float3 *))
{
    const float h = 0.0001f;
    struct float3 k1 = {1, -1, -1};
    struct float3 h1 = {h, -h, -h};
    struct float3 k2 = {-1, -1, 1};
    struct float3 h2 = {-h, -h, h};
    struct float3 k3 = {-1, 1, -1};
    struct float3 h3 = {-h, h, -h};
    struct float3 k4 = {1, 1, 1};
    struct float3 h4 = {h, h, h};

    float3_add_mutate(&h1, p);
    float3_add_mutate(&h2, p);
    float3_add_mutate(&h3, p);
    float3_add_mutate(&h4, p);

    struct float3 col = {0};

    float3_mult_mutate(&k1, f(&h1, &col));
    float3_mult_mutate(&k2, f(&h2, &col));
    float3_mult_mutate(&k3, f(&h3, &col));
    float3_mult_mutate(&k4, f(&h4, &col));

    float3_cpy(p, &k1);
    float3_add_mutate(p, &k2);
    float3_add_mutate(p, &k3);
    float3_add_mutate(p, &k4);

    float3_norm_mutate(p);
}

// raymarch(cam, dir, dts, color) determines if a ray emitted from cam in the direction
//   dir intersects with any scene geometry as described by dts
// requires: dir is normalized [not asserted]
//           cam, dir, dts, color are not NULL
// effects: modifies *color
void raymarch(const struct float3 *cam, struct float3 *dir, float (*dts)(const struct float3 *, struct float3 *), struct float3 *color)
{
    struct float3 pos = {0};
    float3_cpy(&pos, cam);
    for (int i = 0; i < MAX_IT; ++i)
    {
        float dist = dts(&pos, color);
        if (dist < TOUCH_DIST)
        {
            // normal is is found
            struct float3 normal  = {0};
            float3_cpy(&normal, &pos);
            position_to_normal(&normal, dts);

            // Calculate the unit direction vector that points from
            // the point of intersection to the light source
            float3_sub_mutate(&pos, &light_position);
            float3_norm_mutate(&pos);
            float light = fmaxf(0.0f, -float3_dot_prod(&normal, &pos));
            float3_mult_mutate(color, light);
            return;
        }
        else if (float3_dot_prod(&pos, &pos) > SQR_DIST)
        {
            break;
        }
        struct float3 move = {0};
        float3_cpy(&move, dir);
        float3_mult_mutate(&move, dist);
        float3_add_mutate(&pos, &move);
    }
    float3_cpy(color, &backgound_col);
}

// https://raw.githubusercontent.com/lvandeve/lodepng/master/examples/example_encode.c
void encodeOneStep(const char *filename, const unsigned char *image, unsigned width, unsigned height)
{
    /*Encode the image*/
    unsigned error = lodepng_encode32_file(filename, image, width, height);

    /*if there's an error, display it*/
    if (error)
        printf("error %u: %s\n", error, lodepng_error_text(error));
}

int main(int argc, char *argv[])
{
    struct float3 blueGreen = {0.0f, 255.0f, 255.0f};
    struct float3 origin = {.0f, 0.0f, 0.0f};
    struct stage_object sphere = {0};
    sphere.color = &blueGreen;
    sphere.coords_a = &origin;
    sphere.sdf = dist_to_circ;
    sphere.size = 1.0f;
    stage[0] = sphere;

    unsigned char *pixels = malloc((WIDTH * HEIGHT * 4) * sizeof(unsigned char));

    float aspect = WIDTH / HEIGHT;

    for (int x = 0; x < WIDTH; ++x)
    {
        for (int y = 0; y < HEIGHT; ++y)
        {
            struct float3 dir = {(x / WIDTH - 0.5f) * aspect, y / HEIGHT - 0.5f, 1.0f};
            float3_norm_mutate(&dir);
            float3_rotate_x(&dir, HALFPI);

            struct float3 color = {0};

            raymarch(&cam_pos, &dir, dist_to_scene, &color);

            int r = (int)(color.x);
            int g = (int)(color.y);
            int b = (int)(color.z);

            int pixel = (y * WIDTH + x) * 4;
            pixels[pixel] = (char)r;
            pixels[pixel + 1] = (unsigned char)g;
            pixels[pixel + 2] = (unsigned char)b;
            pixels[pixel + 3] = 255;
        }
    }
    char filename[] = "renderOUT.png";
    encodeOneStep(filename, pixels, WIDTH, HEIGHT);
    free(pixels);
    printf("Rendered scene! Check Project folder for image.\n");
    return 0;
}
