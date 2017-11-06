#ifndef INTERP_H
#define INTERP_H

void interpolate(struct Map_info *Map, char* output, double tension,
                 double smoothing, int npmin, int segmax, double dmin,
                 struct bound_box *bbox, double resolution, int threads);
#endif // INTERP_H
