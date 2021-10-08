/** @file point_cloud.cpp
 *  @brief Contains methods for the PointCloud class.
 *  @copyright BSD-3-Clause License
 */

#include <vector>
#include <tuple>
#include "point_cloud.h"

PointCloud::PointCloud()
{

}

PointCloud::~PointCloud()
{

}

/**
 * @brief Save a point cloud to file.
 * @param file_name: Name of the file
 * @return True if successful
 */
bool PointCloud::save(const std::string &file_name)
{
    return open3d::io::WritePointCloud(file_name, _point_cloud);
}

/**
 * @brief Load a point cloud from file.
 * @param file_name: Name of the file
 * @return True if successful
 */
bool PointCloud::load(const std::string &file_name)
{
    return open3d::io::ReadPointCloud(file_name, _point_cloud);
}

/**
 * @brief Remove statistical outliers from the loaded point cloud.
 */
bool PointCloud::remove_outliers()
{
    // Exit if no point cloud were loaded
    if(_point_cloud.IsEmpty())
        return false;

    std::vector<size_t> ind;
    std::tie(std::make_shared<open3d::geometry::PointCloud>(_point_cloud), ind) = _point_cloud.RemoveStatisticalOutliers(10, 0.2);

    return true;
}

/**
 * @brief Launch a visualization window to display the loaded point cloud.
 * @return True if successful
 */
bool PointCloud::visualize()
{
    // Exit if no point cloud were loaded
    if(_point_cloud.IsEmpty())
        return false;

    // Load the point cloud into a vector
    std::vector<std::shared_ptr<const open3d::geometry::Geometry> > point_clouds;
    point_clouds.push_back(std::make_shared<open3d::geometry::PointCloud>(_point_cloud));

    // Display the point cloud
    Eigen::Vector3d lookat_vector(-114, -23, 700);
    Eigen::Vector3d up_vector(0.01, -1, -0.06);
    Eigen::Vector3d front_vector(0, 0.06, -1);
    double zoom = 0.4;
    return open3d::visualization::DrawGeometries(point_clouds, "Point Cloud", 640, 480, 100, 100,
                                                 false, false, false, &lookat_vector, &up_vector, &front_vector, &zoom);
}
