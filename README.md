# r.in.kinect

GRASS GIS module for streaming and importing 3D point cloud, depth, and color data from the Microsoft Kinect for Xbox One (Kinect v2) sensor into GRASS GIS raster and vector layers.

---

## Required Dependencies and Upstream Sources

`r.in.kinect` requires the following software libraries and SDKs:

| Component | Role | Upstream Source / Repository | Ubuntu / Debian Packages | macOS (Homebrew) |
| :--- | :--- | :--- | :--- | :--- |
| **libfreenect2** | Camera driver SDK (USB communication, depth packet decoding, color/depth registration) | [OpenKinect/libfreenect2](https://github.com/OpenKinect/libfreenect2) | Source build (see below) | `brew install libfreenect2` |
| **GRASS GIS** (>= 7.x, 8.x) | GIS application framework & C libraries (`gis`, `raster`, `vector`, `imagery`, `interpfl`, etc.) | [OSGeo/grass](https://github.com/OSGeo/grass) | `grass-dev` `grass-core` | `brew install grass` |
| **PCL** (>= 1.7) | Point Cloud Library (filtering, segmentation, MLS, transforms) | [PointCloudLibrary/pcl](https://github.com/PointCloudLibrary/pcl) | `libpcl-dev` | `brew install pcl` |
| **OpenCV** (>= 3.x) | Computer vision matrix & image containers (`opencv_core`) | [opencv/opencv](https://github.com/opencv/opencv) | `libopencv-dev` | `brew install opencv` |
| **Eigen3** (>= 3.3) | Linear algebra template library | [libeigen/eigen](https://gitlab.com/libeigen/eigen) | `libeigen3-dev` | `brew install eigen` |
| **OpenMP** | Multithreaded grid interpolation | Compilers (GCC/Clang) / LLVM OpenMP | Built into GCC (`libgomp1`) | `brew install libomp` |
| **GDAL** | Geospatial data abstraction library (required by GRASS vector headers) | [OSGeo/gdal](https://github.com/OSGeo/gdal) | `libgdal-dev` | `brew install gdal` |

---

## Detailed Installation of Dependencies

### 1. libfreenect2 (Sensor Driver SDK)

Because Microsoft never released an official cross-platform SDK for the Kinect v2, `libfreenect2` is the open-source driver required to communicate with the sensor.

#### Building libfreenect2 on Ubuntu / Debian:

1. Install prerequisites:
   ```bash
   sudo apt-get install build-essential cmake pkg-config libusb-1.0-0-dev \
       libturbojpeg0-dev libglfw3-dev beignet-opencl-dev libva-dev libopenni2-dev
   ```

2. Clone and build `libfreenect2`:
   ```bash
   git clone https://github.com/OpenKinect/libfreenect2.git
   cd libfreenect2
   mkdir build && cd build
   cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
   make -j$(nproc)
   sudo make install
   ```

3. Setup udev rules for Kinect v2 (Linux only):
   ```bash
   sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

4. Verify driver operation before compiling this module:
   ```bash
   ./bin/Protonect
   ```

### 2. System Libraries (Ubuntu / Debian)

Install all development libraries required by PCL, OpenCV, and GRASS GIS:

```bash
sudo apt-get install \
    cmake \
    g++ \
    libpcl-dev \
    libopencv-dev \
    libeigen3-dev \
    grass-dev \
    grass-core \
    libgdal-dev \
    libpq-dev
```

---

## Building r.in.kinect with CMake

### Standard Build

```bash
# Clone the repository
git clone https://github.com/tangible-landscape/r.in.kinect.git
cd r.in.kinect

# Configure build
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Compile
cmake --build build -j$(nproc)

# Install into GRASS GIS binaries directory
sudo cmake --install build
```

### Build Options

| CMake Option | Default | Description |
| :--- | :--- | :--- |
| `CMAKE_BUILD_TYPE` | `Release` | Build configuration (`Release`, `Debug`, `RelWithDebInfo`) |
| `ENABLE_NATIVE_TUNING` | `ON` | Optimizes binary for host CPU architecture (`-march=native`) |
| `WITH_OPENCL` | `OFF` | Enables OpenCL packet pipeline support in `libfreenect2` |
| `GRASS_BIN_DIR` | Auto (`grass --config path`/bin) | Destination path for the compiled `r.in.kinect` executable |
| `GRASS_DOC_DIR` | Auto (`grass --config path`/docs/html) | Destination path for `r.in.kinect.html` documentation |

### Custom Dependency Prefixes

If `libfreenect2` or `GRASS` are installed in custom non-standard locations, pass their search paths via `CMAKE_PREFIX_PATH`:

```bash
cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="/opt/libfreenect2;/opt/grass"
```

---

## Running the Module

Start GRASS GIS and launch the module:

```bash
grass /path/to/grassdata/location/mapset
r.in.kinect --help
```
