#include <pcl/point_types.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_container.h>


/**
 * Custom Octree Leaf container that keeps color channels
 * separate when averaging
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

/**
 * Custom octree that implements the correct color averaging
 */
// template<typename PointT>
// class ColorOctreePointCloudVoxelCentroid : public pcl::octree::OctreePointCloudVoxelCentroid<PointT, ColorSeparatedLeafContainer<PointT>> {
// public:
//     // Defining base types for conciseness
//     typedef pcl::octree::OctreePointCloudVoxelCentroid<PointT, ColorSeparatedLeafContainer<PointT>> BaseOctreeT;

//     typedef typename BaseOctreeT::LeafNode LeafNode;
//     typedef typename BaseOctreeT::BranchNode BranchNode;

//     // Using base constructor
//     ColorOctreePointCloudVoxelCentroid(double resolution) : BaseOctreeT(resolution) {}

//     // Using base destructor
//     virtual ~ColorOctreePointCloudVoxelCentroid() {}

//     /**
//      * Adding a point to the structure using our container
//      */
//     void addPointIdx(const pcl::uindex_t point_idx) override {
//         // Getting the point from the octree
//         const PointT& point = (*this->input_)[point_idx];

//         // Generating a key for the point
//         pcl::octree::OctreeKey key;
//         this->genOctreeKeyforPoint(point, key);

//         // Adding point to the container
//         LeafNode* leaf = this->createLeaf(key);
//         leaf->getContainer().addPoint(point);
//     }

//     pcl::uindex_t getVoxelCentroids(typename pcl::octree::OctreePointCloud<PointT, ColorSeparatedLeafContainer<PointT>>::AlignedPointTVector& voxel_centroids) const override {
//         // Clearing any previous leaves
//         voxel_centroids.clear();

//         // Adding all the leaves
//         for (auto it = this->leaf_begin(); it != this->leaf_end(); ++it) {
//             PointT centroid;
//             it.getLeafContainer().getCentroid(centroid);
//             voxel_centroids.push_back(centroid);
//         }

//         return static_cast<pcl::uindex_t>(voxel_centroids.size());
//     }
// };


