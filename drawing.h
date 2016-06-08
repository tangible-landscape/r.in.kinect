#ifndef DRAWING_H
#define DRAWING_H

extern "C" {
	#include <grass/gis.h>
	#include <grass/vector.h>
}
#include <vector>

void save_vector(char *name, struct Map_info &Map_draw,
                 struct line_pnts *Points_draw, struct line_cats *Cats_draw,
                 struct bound_box &bbox, struct Cell_head &window,
                 std::vector<double> &draw_x, std::vector<double> &draw_y, std::vector<double> &draw_z,
                 int vect_type, double offset, double zexag);
#endif // DRAWING_H
