#ifndef ANALYSES_H
#define ANALYSES_H

extern "C" {
#include <grass/gis.h>
#include <grass/spawn.h>
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

#endif // ANALYSES_H
