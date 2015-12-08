#ifndef INTERP_H
#define INTERP_H

void interpolate(struct Map_info *Map, char* output, double tension, double smoothing, int npmin, int segmax, double dmin);
#endif // INTERP_H
