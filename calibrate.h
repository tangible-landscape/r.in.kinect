#ifndef CALIBRATE_H
#define CALIBRATE_H
inline double getAngle(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2, const bool in_degree)
{
    // Compute the actual angle
    double rad = v1.normalized().dot (v2.normalized());
    if (rad < -1.0)
        rad = -1.0;
    else if (rad >  1.0)
        rad = 1.0;
    return (in_degree ? acos (rad) * 180.0 / M_PI : acos (rad));
}

template<typename PointT>
void calibrate(boost::shared_ptr<pcl::PointCloud<PointT>> &cloud) {
    // PLANE ESTIMATION
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (0.005);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        G_fatal_error(_("Could not estimate a planar model for the given dataset."));
    }
    Eigen::Vector3f scan_plane(coefficients->values[0],
            coefficients->values[1],
            coefficients->values[2]);
    Eigen::Vector3f plane(0, 0, 1);
    G_important_message(_("Deviation: %f degrees"), getAngle(scan_plane, plane, TRUE));
    Eigen::Vector3f cross = scan_plane.cross(plane).normalized();
    Eigen::Matrix3f u2 = Eigen::Matrix3f::Zero();
    u2(0, 1) = -cross[2];
    u2(0, 2) = cross[1];
    u2(1, 0) = cross[2];
    u2(1, 2) = -cross[0];
    u2(2, 0) = -cross[1];
    u2(2, 1) = cross[0];
    Eigen::Matrix3f mat = Eigen::Matrix3f::Identity().array()  + u2.array() * sin(getAngle(scan_plane, plane, false)) + u2.array()  * u2.array()  * (1 - cos(getAngle(scan_plane, plane, false)));
    std::cout << "calib_matrix=";
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3;j++) {
            std::cout << mat(i, j);
            if (!(i == 2 && j == 2))
                std::cout << ",";
        }
    }
    std::cout << std::endl;

    // For testing only
    /*
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_stat (new pcl::PointCloud<pcl::PointXYZRGB>(512, 424));
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    // Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter (*cloud_filtered_stat);
    cloud_filtered_stat.swap (cloud);
    */
}

Eigen::Matrix4f read_matrix(struct Option *calib_matrix_opt) {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    for (int k = 0; k < 3; k++) {
        for (int l = 0; l < 3; l++) {
            transform(k, l) = atof(calib_matrix_opt->answers[k * 3 + l]);
        }
    }
    return transform;
}

template<typename PointT>
inline void rotate_with_matrix(boost::shared_ptr<pcl::PointCloud<PointT>> &cloud,
                        Eigen::Matrix4f transform) {
    typename pcl::PointCloud<PointT>::Ptr transformed_cloud (new pcl::PointCloud<PointT> (512, 424));
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform);
    transformed_cloud.swap(cloud);
}

#endif // CALIBRATE_H
