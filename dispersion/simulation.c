/* TODO
 */

#include <kilombo.h>

#ifdef SIMULATOR
#include <stdbool.h>
#include <math.h>
#include <limits.h>

#include "simulation.h"
//#include "util.h"
//#include "colors.h"
#include "dispersion.h"  // defines the USERDATA structure


// Walls (svg to csv tool at https://shinao.github.io/PathToPoints/)
size_t walls_len;
double* walls_x;
double* walls_y;
size_t walls_nb_shapes;
#define MAX_NB_SHAPES 10
size_t walls_shapes_end_indexes[MAX_NB_SHAPES];


#include <math.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <kilombo/skilobot.h>


double polygon_area(double* x, double* y, size_t nb_points) {
    double area = 0.;
    size_t j = nb_points - 1;
    for(size_t i = 0; i < nb_points; ++i) {
        area += (x[j*2] + x[i*2]) * (y[j*2] - y[i*2]);
        j = i;
    }
    return area / 2.;
}


double compute_area() {
    double const main_polygon_area = polygon_area(walls_x, walls_y, walls_shapes_end_indexes[0]);
    double excluded_shapes_area[MAX_NB_SHAPES-1];
    double arena_area = main_polygon_area;
    for(size_t s = 1; s < walls_nb_shapes; ++s) {
        size_t const start_idx = walls_shapes_end_indexes[s-1]+1;
        size_t const shape_length = walls_shapes_end_indexes[s] - start_idx;
        excluded_shapes_area[s] = polygon_area(walls_x+start_idx, walls_y+start_idx, shape_length);
        arena_area -= excluded_shapes_area[s];
    }
    return fabs(arena_area);
}

void init_arena() {
    // Load arena file
    char const* arena_filename = get_string_param("arenaFileName", "arenas/arena1.csv");
    FILE* stream = fopen(arena_filename, "r");
    if(stream == NULL) {
        printf("[ERROR] 'arenaFileName' must be set to an existing filename.\n");
        exit(-1);
    }

    // Count the number of lines in arena file
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
    walls_len = 0;
    while(EOF != (fscanf(stream, "%*[^\n]"), fscanf(stream, "%*c")))
        ++walls_len;
#pragma GCC diagnostic pop

    // Reserve memory for arena variables
    walls_x = (double*) calloc(walls_len*2, sizeof(double));
    walls_y = (double*) calloc(walls_len*2, sizeof(double));

    // Initialize arena variables
    fseek(stream, 0, SEEK_SET);
    char line[1024];
    size_t i = 0;
    double min_x = DBL_MAX, min_y = DBL_MAX;
    double max_x = -DBL_MAX, max_y = -DBL_MAX;
    bool has_prev = false;
    double prev_x = 0.;
    double prev_y = 0.;
    double first_x = 0.;
    double first_y = 0.;
    double x = 0.;
    double y = 0.;
    walls_nb_shapes = 0;
    while(fgets(line, 1024, stream)) {
        if(!strcmp(line, "\n")) {
            has_prev = false;
            walls_x[2*i+0] = prev_x;
            walls_y[2*i+0] = prev_y;
            walls_x[2*i+1] = first_x;
            walls_y[2*i+1] = first_y;
            walls_shapes_end_indexes[walls_nb_shapes] = i;
            ++walls_nb_shapes;
            ++i;
            continue;
        }
        char* tmp = strdup(line);
        char const* tok_x = strtok(tmp, ",");
        char const* tok_y = strtok(NULL, ",\n");
        x = atof(tok_x);
        y = atof(tok_y);
        free(tmp);

        if(has_prev) {
            walls_x[2*i+0] = prev_x;
            walls_y[2*i+0] = prev_y;
            walls_x[2*i+1] = x;
            walls_y[2*i+1] = y;
            ++i;
        } else {
            first_x = x;
            first_y = y;
        }
        prev_x = x;
        prev_y = y;
        has_prev = true;
        if(x < min_x)
            min_x = x;
        if(x > max_x)
            max_x = x;
        if(y < min_y)
            min_y = y;
        if(y > max_y)
            max_y = y;
    }
    walls_shapes_end_indexes[walls_nb_shapes] = i;
    ++walls_nb_shapes;
    walls_x[2*i+0] = prev_x;
    walls_y[2*i+0] = prev_y;
    walls_x[2*i+1] = first_x;
    walls_y[2*i+1] = first_y;

//    {
//        printf("Coords:\n-------\n");
//        size_t start_idx = 0;
//        for(size_t s = 0; s < walls_nb_shapes; ++s) {
//            size_t const shape_length = walls_shapes_end_indexes[s] - start_idx + 1;
//            for(i = 0; i < shape_length; i++) {
//                printf("%f,%f\n", walls_x[2*(start_idx+i)], walls_y[2*(start_idx+i)]);
//            }
//            start_idx += shape_length;
//            printf("\n");
//        }
//        printf("-------\n\n");
//    }

    // Normalize arena coords
    printf("Raw area: %f\n", compute_area());
    double const arena_width = (double)simparams->display_w / simparams->display_scale * 0.90f;
    double const arena_half_width = arena_width / 2.f;
    double const arena_height = (double)simparams->display_h / simparams->display_scale * 0.90f;
    double const arena_half_height = arena_height / 2.f;
    for(i = 0; i < walls_len*2; i++) {
        walls_x[i] = (walls_x[i] - min_x) / (max_x - min_x) * arena_width - arena_half_width;
        walls_y[i] = (walls_y[i] - min_y) / (max_y - min_y) * arena_height - arena_half_height;
        //printf("# %f %f\n", walls_x[i], walls_y[i]);
    }
    double const normalized_area = compute_area();
    printf("Normalized area: %f\n", normalized_area);

    // Homothetie to normalize arena surface
    double const target_area = get_float_param("arenaNormalizedArea", 400000.);
    double const scaling_factor = sqrt(target_area / normalized_area);
    for(i = 0; i < walls_len*2; i++) {
        walls_x[i] = 0. + scaling_factor * (walls_x[i] - 0.);
        walls_y[i] = 0. + scaling_factor * (walls_y[i] - 0.);
    }
    printf("Scaled area: %f\n", compute_area());

//    {
//        printf("Scaled coords:\n-------\n");
//        size_t start_idx = 0;
//        for(size_t s = 0; s < walls_nb_shapes; ++s) {
//            //printf("%d %d\n", s, walls_shapes_end_indexes[s]);
//            size_t const shape_length = walls_shapes_end_indexes[s] - start_idx + 1;
//            for(i = 0; i < shape_length; i++) {
//                printf("%f,%f\n", walls_x[2*(start_idx+i)], walls_y[2*(start_idx+i)]);
//            }
//            start_idx += shape_length;
//            printf("\n");
//        }
//        printf("-------\n\n");
//    }

}



void global_setup() {
    init_arena();
}


/* provide a text string for the simulator status bar about this bot */
static char botinfo_buffer[10000];
char *botinfo(void) {
    char *p = botinfo_buffer;
    p += sprintf (p, "ID: %d   nb_neighbours: %d \n", kilo_uid, mydata->N_Neighbors);

//    p += sprintf (p, "%d neighbors: ", mydata->N_Neighbors);
//    for (size_t i = 0; i < mydata->N_Neighbors; i++)
//        p += sprintf (p, "%d ", mydata->neighbors[i].ID);

    return botinfo_buffer;
}



int pnpoly(int nvert, double *vertx, double *verty, double testx, double testy) {
    int i, j, c = 0;
    for (i = 0, j = nvert-1; i < nvert; j = i++) {
        if ( ((verty[i*2]>testy) != (verty[j*2]>testy)) &&
                (testx < (vertx[j*2]-vertx[i*2]) * (testy-verty[i*2]) / (verty[j*2]-verty[i*2]) + vertx[i*2]) )
            c = !c;
    }
    return c;
}

coord_t closest_wall(double x, double y, double x1, double y1, double x2, double y2) {
    double const a = x - x1;
    double const b = y - y1;
    double const c = x2 - x1;
    double const d = y2 - y1;

    double const dot = a * c + b * d;
    double const len_sq = c * c + d * d;
    double param = -1;
    if(len_sq != 0) // In case of 0 length line
        param = dot / len_sq;

    double xx, yy;
    if (param < 0) {
        xx = x1;
        yy = y1;
    } else if (param > 1) {
        xx = x2;
        yy = y2;
    } else {
        xx = x1 + param * c;
        yy = y1 + param * d;
    }

    coord_t res = {xx, yy};
    return res;
}


int16_t obstacles_walls(double x, double y, double * dx, double * dy) {
    // Check if bot is inside the main shape
    bool inside_arena = pnpoly(walls_shapes_end_indexes[0], walls_x, walls_y, x, y);
    // Check if bot is outside of the excluded shapes
    if(inside_arena) {
        for(size_t i = 1; i < walls_nb_shapes; i++) {
            size_t const start_idx = walls_shapes_end_indexes[i-1]+1;
            size_t const shape_length = walls_shapes_end_indexes[i] - start_idx;
            inside_arena &= !pnpoly(shape_length, walls_x+start_idx*2, walls_y+start_idx*2, x, y);
        }
    }

    if(!inside_arena) {
        //printf("OBSTACLE !!\n");
        // Find closest wall
        double closest_d = DBL_MAX;
        for(size_t i = 0; i < walls_len; i++) {
            coord_t const c = closest_wall(x, y, walls_x[2*i], walls_y[2*i], walls_x[2*i+1], walls_y[2*i+1]);
            //printf("# %f %f %f %f\n", walls_x[2*i], walls_y[2*i], walls_x[2*i+1], walls_y[2*i+1]);
            double const _dx = x - c.x;
            double const _dy = y - c.y;
            double const d = sqrt(_dx * _dx + _dy * _dy);
            if(d < closest_d) {
                closest_d = d;
                if(i > walls_shapes_end_indexes[0]) {
                    *dx = -_dx;
                    *dy = -_dy;
                } else {
                    *dx = -_dx;
                    *dy = -_dy;
                }
            }
        }
        return 1;
    } else {
        return 0;
    }
}

#endif


// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
