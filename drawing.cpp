extern "C" {
#include <grass/gis.h>
#include <grass/vector.h>
#include <grass/glocale.h>
}
#include <vector>

void save_vector(char *name, struct Map_info &Map_draw,
                 struct line_pnts *Points_draw, struct line_cats *Cats_draw,
                 struct bound_box &bbox, struct Cell_head &window,
                 std::vector<double> &draw_x, std::vector<double> &draw_y, std::vector<double> &draw_z,
                 int vect_type, double offset, double zexag)
{
    double scale = ((window.north - window.south) / (bbox.N - bbox.S) +
                    (window.east - window.west) / (bbox.E - bbox.W)) / 2;

    Vect_reset_line(Points_draw);
    Vect_reset_cats(Cats_draw);

    if (Vect_open_new(&Map_draw, name, WITHOUT_Z) < 0)
        G_fatal_error(_("Unable to create vector map <%s>"), name);
    // draw line
    double x, y, z;
    for (int j = 0; j < draw_x.size();j++) {
        x = (draw_x[j] - bbox.W) * scale + window.west;
        y = (draw_y[j] - bbox.S) * scale + window.south;
        z = (draw_z[j] - bbox.B) * scale / zexag + offset;
        if (vect_type == GV_POINT) {
            if(j == draw_x.size() - 1)
                Vect_append_point(Points_draw, x, y, z);
        }
        else
            Vect_append_point(Points_draw, x, y, z);
    }
    if (vect_type == GV_AREA) {
        x = (draw_x[0] - bbox.W) * scale + window.west;
        y = (draw_y[0] - bbox.S) * scale + window.south;
        z = (draw_z[0] - bbox.B) * scale / zexag + offset;
        Vect_append_point(Points_draw, x, y, z);
        Vect_write_line(&Map_draw, GV_BOUNDARY, Points_draw, Cats_draw);
        double cx, cy;
        Vect_get_point_in_poly(Points_draw, &cx, &cy);
        Vect_reset_line(Points_draw);
        Vect_reset_cats(Cats_draw);
        Vect_append_point(Points_draw, cx, cy, 0);
        Vect_cat_set(Cats_draw, 1, 1);
        Vect_write_line(&Map_draw, GV_CENTROID, Points_draw, Cats_draw);
    }
    else {
        Vect_cat_set(Cats_draw, 1, 1);
        Vect_write_line(&Map_draw, vect_type, Points_draw, Cats_draw);
    }
    Vect_build(&Map_draw);
    Vect_close(&Map_draw);
}
