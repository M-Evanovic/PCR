#include "fpfh/fpfh.h"

namespace pcr {

FPFH::FPFH() {
    source_descriptors_.reset(new FPFHCloudType());
    target_descriptors_.reset(new FPFHCloudType());
    LoadRosParam();
}

FPFH::~FPFH() {}

void FPFH::LoadRosParam() {
    nh.param<double>("voxel_size", voxel_size, 0.5);
    nh.param<double>("FPFH/normal_radius_", normal_radius_, 0.5);
    nh.param<double>("FPFH/fpfh_radius_", fpfh_radius_, 0.75);
    nh.param<int>("FPFH/fpfh_match_threshold", fpfh_match_threshold, 50);
}

FPFHCloudPtr FPFH::ComputeFPFHFeatures(const CloudPtr &input_cloud, NormalCloudType &normals,
                                       double normal_search_radius, double fpfh_search_radius) {
    // define some intermediate variables
    NormalCloudPtr estimated_normals(new NormalCloudType());
    FPFHCloudPtr descriptors(new FPFHCloudType());

    // estimate normals using pcl function
    pcl::NormalEstimationOMP<PointType, pcl::Normal> normal_estimator;
    normal_estimator.setInputCloud(input_cloud);
    normal_estimator.setRadiusSearch(normal_search_radius);
    pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
    normal_estimator.setSearchMethod(kdtree);
    normal_estimator.compute(*estimated_normals);

    normals = *estimated_normals;

    // estimate FPFH using pcl function
    fpfh_estimation_->setInputCloud(input_cloud);
    fpfh_estimation_->setInputNormals(estimated_normals);
    fpfh_estimation_->setSearchMethod(kdtree);
    fpfh_estimation_->setRadiusSearch(fpfh_search_radius);
    fpfh_estimation_->compute(*descriptors);

    return descriptors;
}

void FPFH::SetFeaturePair() {
    if (normal_radius_ > fpfh_radius_) {
        std::cout << "normal_radius = " << normal_radius_ << "\t\t\t"
                  << "fpfh_radius = " << fpfh_radius_ << std::endl;
        throw std::invalid_argument("[FPFHManager]: Normal radius should be lower than fpfh radius");
    }

    pcl::PointCloud<pcl::Normal>::Ptr source_normals_raw(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::Normal>::Ptr target_normals_raw(new pcl::PointCloud<pcl::Normal>());

    source_descriptors_ = ComputeFPFHFeatures(source_feature, *source_normals_raw, normal_radius_, fpfh_radius_);
    target_descriptors_ = ComputeFPFHFeatures(target_feature, *target_normals_raw, normal_radius_, fpfh_radius_);
    
    pcr::FeatureMatcher matcher;
    corr_ = matcher.CalculateCorrespondences(*source_feature, *target_feature,
                                             *source_descriptors_, *target_descriptors_,
                                             true, true, true, 0.95);
    
    if (corr_.size() < fpfh_match_threshold) {
        std::cout << "FPFH Match Size is too low ===>> Skip" << std::endl;
        return;
    }

    source_matched_.resize(3, corr_.size());
    target_matched_.resize(3, corr_.size());
    target_normals_.resize(3, corr_.size());
    
    for (int i = 0; i < corr_.size(); ++i) {
        auto src_idx = corr_[i].first;
        auto tgt_idx = corr_[i].second;
        source_matched_.col(i) << source_feature->points[src_idx].x, source_feature->points[src_idx].y,
            source_feature->points[src_idx].z;
        target_matched_.col(i) << target_feature->points[tgt_idx].x, target_feature->points[tgt_idx].y,
            target_feature->points[tgt_idx].z;

        target_normals_.col(i) << target_normals_raw->points[tgt_idx].normal_x,
            target_normals_raw->points[tgt_idx].normal_y, target_normals_raw->points[tgt_idx].normal_z;
    }

    pcr::Eigen2Pcl(source_matched_, source_matched_pcl_);
    pcr::Eigen2Pcl(target_matched_, target_matched_pcl_);

    std::cout << std::endl;
}

void FPFH::Run() {
    pcr::TicToc fpfh_time;
    pcr::Voxelize<PointType>(source_cloud, source_feature, voxel_size);
    pcr::Voxelize<PointType>(target_cloud, target_feature, voxel_size);
    // source_feature = source_cloud;
    // target_feature = target_cloud;

    SetFeaturePair();
    
    std::cout << "FPFH matched size = " << corr_.size() << std::endl;
    std::cout << "FPFH cost: " << fpfh_time.Toc() << " ms" << std::endl;
    std::cout << "FPFH Finished" << std::endl << std::endl;
}

} // namespace pcr
