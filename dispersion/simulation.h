
#ifndef DISPERSION_H_
#define DISPERSION_H_

#define init_float(X) {X = get_float_param(#X , X); }
#define init_int(X) {X = get_int_param(#X, X); }

typedef struct {
    double x;
    double y;
} coord_t;

double polygon_area(double* x, double* y, size_t nb_points);
double compute_area();
void init_arena();
void global_setup();
char *botinfo(void);
int pnpoly(int nvert, double *vertx, double *verty, double testx, double testy);
coord_t closest_wall(double x, double y, double x1, double y1, double x2, double y2);
int16_t obstacles_walls(double x, double y, double * dx, double * dy);

#endif

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
