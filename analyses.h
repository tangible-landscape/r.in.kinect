#ifndef ANALYSES_H
#define ANALYSES_H

extern "C" {
#include <grass/gis.h>
#include <grass/raster.h>
#include <grass/vector.h>
#include <grass/spawn.h>
#include <grass/glocale.h>
}

inline
int contours(const char *map, const char *output, float step)
{
    char buf[1024];
    const char *argv[8];
    int argc = 0;


    argv[argc++] = "r.contour";
    argv[argc++] = "-t";
    argv[argc++] = "--o";

    sprintf(buf, "input=%s", map);
    argv[argc++] = G_store(buf);
    sprintf(buf, "output=%s", output);
    argv[argc++] = G_store(buf);
    sprintf(buf, "step=%f", step);
    argv[argc++] = G_store(buf);
    argv[argc++] = NULL;

    return G_vspawn_ex(argv[0], argv);
}

inline
void set_default_color(char* raster) {
/* colortable for elevations */
    struct Colors colors;
    struct FPRange range;
    double zmin, zmax;
    Rast_init_colors(&colors);
    Rast_read_fp_range(raster, "", &range);
    Rast_get_fp_range_min_max(&range, &zmin, &zmax);

    double zstep = (FCELL) (zmax - zmin) / 5.;
    for (int j = 1; j <= 5; j++) {
        FCELL data1 = (FCELL) (zmin + (j - 1) * zstep);
        FCELL data2 = (FCELL) (zmin + j * zstep);
        switch (j) {
          case 1:
              Rast_add_f_color_rule(&data1, 50, 121, 70,
                                    &data2, 90, 148, 80, &colors);
          break;
          case 2:
              Rast_add_f_color_rule(&data1, 90, 148, 80,
                                  &data2, 148, 174, 92, &colors);
          break;
          case 3:
              Rast_add_f_color_rule(&data1, 148, 174, 92,
                                    &data2, 224, 205, 103, &colors);
          break;
          case 4:
              Rast_add_f_color_rule(&data1, 224, 205, 103,
                                    &data2, 186, 151, 74, &colors);
          break;
          case 5:
              Rast_add_f_color_rule(&data1, 186, 151, 74,
                                    &data2, 159, 100, 44, &colors);
          break;
          }
    }
    const char *mapset = G_find_file("cell", raster, "");
    Rast_write_colors(raster, mapset, &colors);
    Rast_quantize_fp_map_range(raster, mapset,
                               (DCELL) zmin - 0.5, (DCELL) zmax + 0.5,
                               (CELL) (zmin - 0.5), (CELL) (zmax + 0.5));
}

inline
int equalized(const char *map)
{
    char buf[1024];
    const char *argv[5];
    int argc = 0;


    argv[argc++] = "r.colors";
    argv[argc++] = "-e";
    argv[argc++] = "--q";
    argv[argc++] = "color=elevation";

    sprintf(buf, "map=%s", map);
    argv[argc++] = G_store(buf);
    argv[argc++] = NULL;

    return G_vspawn_ex(argv[0], argv);
}
#endif // ANALYSES_H
