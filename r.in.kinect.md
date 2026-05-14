## DESCRIPTION

*r.in.kinect* imports data scanned by a depth camera as a GRASS raster or
vector map. It is the scanning backend of the
[Tangible Landscape](https://tangible-landscape.github.io/) project, in which a
depth camera mounted above a physical model continuously captures the model's
surface and turns it into a GRASS raster representing topography.

This branch supports the **Orbbec Femto Bolt** sensor through the Orbbec SDK.
Other branches of the module support the Microsoft Azure Kinect DK and the
Kinect for Xbox One.

The module captures a point cloud from the sensor, applies a series of optional
filtering and transformation steps, and writes the result as one or more of:

- a raster map (**output**) representing the scanned surface,
- RGB raster maps from the color camera (**color_output**),
- a vector point map (**vector**),
- a binary PLY point-cloud file (**ply**).

### Processing pipeline

The point cloud passes through the following steps, in order. Each step is
skipped when its options are not set:

1. Capture **numscan** frames and merge them into one cloud.
2. Apply the calibration matrix (**calib_matrix**) to correct sensor tilt.
3. Filter points by distance from the sensor (**zrange**).
4. Rotate the cloud around the vertical axis (**rotate**).
5. Crop a fixed box measured from the scan center (**trim**), or trim empty
   margins automatically (**trim_tolerance**).
6. Remove statistical outliers.
7. Smooth the surface (**smooth_radius**).
8. Bin or interpolate the points onto a raster (**method**,
   **interpolation_method**, **resolution**).
9. Georeference the result to a GRASS region (**region**, **raster**,
   **zexag**).

### Coordinate system

The sensor is assumed to look down at an approximately horizontal surface. The
*z* coordinate is the distance from the sensor, so smaller *z* values are closer
to the camera. After calibration the scan plane is leveled, and elevations are
scaled and shifted to fit the horizontal extent and vertical range of the target
region. Distances given in **zrange** and **trim** are expressed in centimeters.

## NOTES

### Calibration

Before scanning, the module is typically run twice in calibration mode:

- **-c** (sensor calibration) fits a plane to the point cloud and estimates the
  tilt of the sensor relative to the scanned surface and its height above it. It
  prints `angle_deviation`, a nine-value `calib_matrix`, and `height`. Pass the
  nine values back as **calib_matrix** in subsequent runs to level the scan.
- **-m** (model position calibration) detects the physical model on the table
  and prints `bbox=N,S,E,W` in centimeters from the scan center. Use these
  values as **trim** to crop the scan to the model.

The **-c** and **-m** flags are mutually exclusive.

### Continuous scanning and runtime control

With **-l** the module keeps scanning in a loop instead of exiting after a
single scan. While looping, parameters can be changed at runtime: send the
process the `SIGUSR1` signal and it will read `key=value` lines from standard
input (one per line, terminated by an empty line). Most options can be updated
this way, including `output`, `resolution`, `zrange`, `trim`, `rotate`,
`zexag`, `method`, and `camera_resolution`. Changing `camera_resolution`
reinitializes the sensor. Three additional control keywords are recognized:

- `pause` / `resume` — stop and restart scanning,
- `resume_once` — process a single scan and pause again.

This runtime protocol is how the GRASS
[Tangible Landscape plugin](https://github.com/tangible-landscape/grass-tangible-landscape)
drives the module.

If **signal_file** is set, an empty file with that name is created after each
scan cycle, which other programs can watch to detect when a new result is ready.

### Drawing with a laser pointer

If **draw_output** is given, the module looks for the brightest spot in the
color image (with combined RGB brightness above **draw_threshold**) and records
its position. Consecutive detections are collected and written to
**draw_output** as a point, line, or area depending on **draw**. This mode
requires the color camera.

### Surface reconstruction

**method** controls how points are turned into raster cells: `mean`, `min`, and
`max` aggregate the points falling into each cell, while `interpolation` fills
the raster from the points. When **method** is `interpolation`,
**interpolation_method** selects between inverse distance weighting (`idw`) and
regularized splines (`splines`); spline interpolation can be parallelized with
**nprocs_interpolation**.

### Sensor info

The **-i** flag prints sensor information and exits without scanning.

## EXAMPLES

Calibrate the sensor tilt and height above the table:

```sh
r.in.kinect -c
```

Calibrate the position of the physical model:

```sh
r.in.kinect -m
```

Scan a physical model into a raster, georeferenced to the current region:

```sh
r.in.kinect output=scan resolution=0.002 \
  zrange=80,95 trim=20,20,30,30
```

Import the scan together with color rasters from a 1080p color camera:

```sh
r.in.kinect output=scan color_output=scan_color \
  resolution=0.002 color_resolution=0.002 camera_resolution=1080P
```

Scan continuously, matching the result to the extent and resolution of an
existing raster map:

```sh
r.in.kinect -l output=scan resolution=0.002 raster=elevation
```

Export the raw point cloud to a binary PLY file:

```sh
r.in.kinect ply=scan.ply
```

## REFERENCES

- Petrasova, A., Harmon, B., Petras, V., Tabrizian, P., Mitasova, H. (2018).
  *Tangible Modeling with Open Source GIS*. Second edition. Springer.
  <https://doi.org/10.1007/978-3-319-89303-7>
- Tangible Landscape project: <https://tangible-landscape.github.io/>

## SEE ALSO

*[r.in.xyz](r.in.xyz.md)*,
*[r.in.pdal](r.in.pdal.md)*,
*[v.in.lidar](v.in.lidar.md)*,
*[r.contour](r.contour.md)*,
*[r.fillnulls](r.fillnulls.md)*,
*[v.surf.rst](v.surf.rst.md)*

## AUTHORS

Anna Petrasova, NCSU Center for Geospatial Analytics

Vaclav Petras, NCSU Center for Geospatial Analytics

Orbbec Femto Bolt sensor support was contributed by Everett Tucker.
