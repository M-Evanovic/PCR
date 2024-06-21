#include <pcl/common/transforms.h>
#include <pcl/common/io.h>

#include "utility.h"
#include "patchworkpp.hpp"
#include "fpfh/fpfh.h"
#include "quatro/quatro.hpp"
#include "teaser/teaser.h"
#include "fitter.h"

namespace pcr {
class Register {
public:
    Register();
    ~Register();
    void SetSourcePointCloud(PointCloudType src_cloud);
    void SetTargetPointCloud(PointCloudType tgt_cloud);
    Eigen::Matrix4d GetTransformtionMatrix() {
        return output;
    }
    void Run();

private:
    void LoadRosParams();
    void SetParams(
            double noise_bound_of_each_measurement, double square_of_the_ratio_btw_noise_and_noise_bound,
            double estimating_scale, int num_max_iter, double control_parameter_for_gnc,
            double rot_cost_thr, RobustRegistrationSolver::Params &params);
    void SetParams(
            double noise_bound_of_each_measurement, double square_of_the_ratio_btw_noise_and_noise_bound,
            double estimating_scale, int num_max_iter, double control_parameter_for_gnc,
            double rot_cost_thr, const string& reg_type_name, Quatro<PointType, PointType>::Params &params);

private:
    ros::NodeHandle nh;

    PatchWorkpp<PointType> groundSeperator;

    std::string pcd_path;

    pcr::FPFH fpfh;
    int fpfh_match_threshold;

    int fpfh_mode = 1;
    int registration_mode = 1;

    PointCloudType source_cloud_raw;
    PointCloudType target_cloud_raw;
    PointCloudType source_ground;
    PointCloudType source_without_ground;
    PointCloudType target_ground;
    PointCloudType target_without_ground;

    bool   estimating_scale;
    int    num_max_iter;
    double noise_bound, noise_bound_coeff, gnc_factor, rot_cost_diff_thr;

    pcl::PCDWriter pcd_writer;

    Eigen::Matrix4d output = Eigen::Matrix4d::Identity();
};

} // namespace pcr