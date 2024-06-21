#ifndef _FEATURE_MATCHER_H_
#define _FEATURE_MATCHER_H_

#include "utility.h"

namespace pcr {

class FeatureMatcher {
   public:
    FeatureMatcher() = default;

    std::vector<std::pair<int, int>> CalculateCorrespondences(
        PointCloudType &source_cloud, PointCloudType &target_cloud, FPFHCloudType &source_features,
        FPFHCloudType &target_features, bool use_absolute_scale = true, bool use_crosscheck = true,
        bool use_tuple_test = true, float tuple_scale = 0);

    typedef std::vector<Eigen::Matrix<float, Eigen::Dynamic, 1>> Feature;
    typedef flann::Index<flann::L2<float>> KDTree;

   private:
    template <typename T>
    void BuildKDTree(const std::vector<T> &data, KDTree *tree);

    template <typename T>
    void SearchKDTree(KDTree *tree, const T &input, std::vector<int> &indices, std::vector<float> &dists, int nn);

    void AdvancedMatching(bool use_crosscheck, bool use_tuple_test, float tuple_scale);

    void NormalizePoints(bool use_absolute_scale);

    std::vector<std::pair<int, int>> corres_;
    std::vector<PointCloudType> pointcloud_;
    std::vector<Feature> feature_;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> means_;  // for normalization
    float global_scale_;
};

} // namespace pcr
#endif