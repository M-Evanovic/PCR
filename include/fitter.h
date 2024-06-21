#pragma once

#include <iostream>
#include <Eigen/Core>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include "utility.h"

namespace pcr {

class Fitter {
public:
    Fitter();
    ~Fitter();

    void SetSourcePointCloud(PointCloudType src_cloud) {
        pcr::Pcl2Eigen<PointType>(src_cloud, source_cloud_matrix);
    }
    void SetTargetPointCloud(PointCloudType tgt_cloud) {
        pcr::Pcl2Eigen<PointType>(tgt_cloud, target_cloud_matrix);
    }

    Eigen::Matrix4d GetTransformtionMatrix() {
        return transform_matrix;
    }

    void Best_Fit(void);

private:
    Eigen::MatrixXd source_cloud_matrix;
    Eigen::MatrixXd target_cloud_matrix;

    Eigen::Vector3d translation_vector;
    Eigen::Matrix3d rotation_matrix;
    Eigen::Matrix4d transform_matrix = Eigen::MatrixXd::Identity(4,4);
};

} // namespace pcr

