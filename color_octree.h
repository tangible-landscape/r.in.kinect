#include <pcl/point_types.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_container.h>


/**
 * Custom Octree Leaf container that maintains separate color
 * channels when averaging
 * Each instance represents a centroid with added points averaged together
 */

template<typename PointT>
class ColorSeparatedLeafContainer : public pcl::octree::OctreeContainerPointIndices {
public:
    /* Initializing sums for each channel's average and num points */
    ColorSeparatedLeafContainer() :
        point_counter(0),
        x_sum(0), y_sum(0), z_sum(0),
        r_sum(0), g_sum(0), b_sum(0) {}
    
    /**
     * Adds a point to the container
     * @param point a reference to the point to add
     */
    void addPoint(const PointT& point) {
        // Incrementing position
        x_sum += point.x;
        y_sum += point.y;
        z_sum += point.z;

        // Incrementing Color
        r_sum += point.r;
        g_sum += point.g;
        b_sum += point.b;

        // Incrementing points
        point_counter++;
    }

    /**
     * Returns the centroid of the container by reference
     */
    void getCentroid(PointT& centroid) const {
        // No points
        if (!point_counter) return;

        // Defining inverse to limit division
        float inv_n = 1.0f / static_cast<float>(point_counter);

        // Averaging xyz
        centroid.x = x_sum * inv_n;
        centroid.y = y_sum * inv_n;
        centroid.z = z_sum * inv_n;

        // Averaging rgb
        centroid.r = r_sum * inv_n;
        centroid.g = g_sum * inv_n;
        centroid.b = b_sum * inv_n;
    }

    /**
     * Resetting the container
     */
    void reset() {
        point_counter = 0;
        x_sum = y_sum = z_sum = 0.0f;
        r_sum = g_sum = b_sum = 0.0f;
    }

private:
    std::size_t point_counter;
    float x_sum, y_sum, z_sum;
    float r_sum, g_sum, b_sum;
};
