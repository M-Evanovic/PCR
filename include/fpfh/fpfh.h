#ifndef _FPFH_H_
#define _FPFH_H_

#include "utility.h"
#include "feature_matcher.h"

namespace pcr {

class FPFH{
public:
    FPFH();
    ~FPFH();

    void SetSourcePointCloud(PointCloudType src_cloud) {
        *source_cloud = src_cloud;
    }
    void SetTargetPointCloud(PointCloudType tgt_cloud) {
        *target_cloud = tgt_cloud;
    }

    int GetMatchedSize() {
        return corr_.size();
    }

    std::vector<std::pair<int, int>> GetCorrespondences() {
        return corr_;
    }

    pcl::PointCloud<PointType> GetSrcKps(){
        return source_matched_pcl_;
    }
    pcl::PointCloud<PointType> GetTgtKps(){
        return target_matched_pcl_;
    }

    void Run();

private:
    ros::NodeHandle nh;

    void LoadRosParam();

    FPFHCloudPtr ComputeFPFHFeatures(const CloudPtr &input_cloud, NormalCloudType &normals,
                                     double normal_search_radius = 0.03, double fpfh_search_radius = 0.05);

    void SetFeaturePair();

private:
    std::string pcd_path;

    pcl::FPFHEstimationOMP<PointType, NormalPointType, FPFHType>::Ptr fpfh_estimation_{new pcl::FPFHEstimationOMP<PointType, pcl::Normal, FPFHType>()};

    FPFHCloudPtr source_descriptors_;
    FPFHCloudPtr target_descriptors_;

    PointCloudType source_matched_pcl_;
    PointCloudType target_matched_pcl_;
    
    CloudPtr source_cloud{new PointCloudType()};
    CloudPtr target_cloud{new PointCloudType()};

    CloudPtr source_feature{new PointCloudType()};
    CloudPtr target_feature{new PointCloudType()};

    double normal_radius_;
    double fpfh_radius_;

    double voxel_size;

    std::vector<std::pair<int, int>> corr_;
    int fpfh_match_threshold;

    Eigen::Matrix<double, 3, Eigen::Dynamic> source_matched_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> target_matched_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> target_normals_;
};

} // namespace pcr
#endif