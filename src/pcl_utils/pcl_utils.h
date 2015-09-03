//
// Created by Dan Butler on 6/22/15.
//

#ifndef PICARD_ALGORITHMS_H
#define PICARD_ALGORITHMS_H

#include "common.h"

#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/voxel_grid.h>

namespace picard
{

void downsample(const PointCloudPtr &input_cloud, PointCloud output_cloud, float leaf_size) ;

bool extract(const PointCloudConstPtr &input_cloud, const pcl::PointIndices::ConstPtr &indices, const PointCloud &output_cloud, bool negative);

bool extract(const PointCloudConstPtr &input_cloud, const pcl::PointIndices &indices, PointCloud &output_cloud);

bool getLargestPlaneComponent(const PointCloudPtr &input_cloud, const pcl::ModelCoefficients::Ptr &coeffs, const pcl::PointIndices::Ptr &planar_cluster_inliers);

void getPointsAbove2dHull(const PointCloudConstPtr &plane_outliers, const PointCloud &cloud_hull, const PointCloudPtr &points_above_plane);

void euclideanCluster(const PointCloudConstPtr &input_cloud, std::vector<pcl::PointIndices> &cluster_indices);

void interpretTableScene(const PointCloudConstPtr &input_cloud, const ModelCoefficientsPtr &coeffs, const PointCloudPtr &plane_points, const PointCloudPtr& cloud_hull, std::vector<PointCloudPtr> &object_clouds);

}

#endif //PICARD_ALGORITHMS_H