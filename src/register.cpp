#include "register.h"

namespace pcr {

Register::Register() {}

Register::~Register() {}

void Register::LoadRosParams() {
    nh.param<std::string>("pcd_path", pcd_path, "./");
    nh.param<int>("FPFH/fpfh_match_threshold", fpfh_match_threshold, 50);
    nh.param<int>("/fpfh_mode", fpfh_mode, 1);
    nh.param<int>("/registration_mode", registration_mode, 1);
    nh.param<bool>("/quatro/estimating_scale", estimating_scale, false);
    nh.param<int>("/quatro/rotation/num_max_iter", num_max_iter, 50);
    nh.param<double>("/quatro/noise_bound", noise_bound, 0.25);
    nh.param<double>("/quatro/noise_bound_coeff", noise_bound_coeff, 0.99);
    nh.param<double>("/quatro/rotation/gnc_factor", gnc_factor, 1.39);
    nh.param<double>("/quatro/rotation/rot_cost_diff_thr", rot_cost_diff_thr, 0.0001);

}

void Register::SetParams(
        double noise_bound_of_each_measurement, double square_of_the_ratio_btw_noise_and_noise_bound,
        double estimating_scale, int num_max_iter, double control_parameter_for_gnc,
        double rot_cost_thr, RobustRegistrationSolver::Params &params) {
    params.noise_bound                   = noise_bound_of_each_measurement;
    params.cbar2                         = square_of_the_ratio_btw_noise_and_noise_bound;
    params.estimate_scaling              = estimating_scale;
    params.rotation_max_iterations       = num_max_iter;
    params.rotation_gnc_factor           = control_parameter_for_gnc;
    params.rotation_estimation_algorithm = RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    params.rotation_cost_threshold       = rot_cost_thr;
}

void Register::SetParams(
        double noise_bound_of_each_measurement, double square_of_the_ratio_btw_noise_and_noise_bound,
        double estimating_scale, int num_max_iter, double control_parameter_for_gnc,
        double rot_cost_thr, const string& reg_type_name, Quatro<PointType, PointType>::Params &params) {
    params.noise_bound                   = noise_bound_of_each_measurement;
    params.cbar2                         = square_of_the_ratio_btw_noise_and_noise_bound;
    params.estimate_scaling              = estimating_scale;
    params.rotation_max_iterations       = num_max_iter;
    params.rotation_gnc_factor           = control_parameter_for_gnc;
    params.rotation_estimation_algorithm = Quatro<PointType, PointType>::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    params.rotation_cost_threshold       = rot_cost_thr;

    params.reg_name                  = reg_type_name;
    if (reg_type_name == "Quatro") {
        params.inlier_selection_mode = Quatro<PointType, PointType>::INLIER_SELECTION_MODE::PMC_HEU;
    } else { params.inlier_selection_mode = Quatro<PointType, PointType>::INLIER_SELECTION_MODE::PMC_EXACT; }
}

void Register::SetSourcePointCloud(PointCloudType src_cloud) {
    source_cloud_raw = src_cloud;
    groundSeperator.EstimateGround(source_cloud_raw, source_ground, source_without_ground);
}

void Register::SetTargetPointCloud(PointCloudType tgt_cloud) {
    target_cloud_raw = tgt_cloud;
    groundSeperator.EstimateGround(target_cloud_raw, target_ground, target_without_ground);
}

void Register::Run() {
    LoadRosParams();
    // Calculate FPFH
    if (fpfh_mode == 1) {
        fpfh.SetSourcePointCloud(source_without_ground);
        fpfh.SetTargetPointCloud(target_without_ground);
    } else if (fpfh_mode == 2) {
        fpfh.SetSourcePointCloud(source_cloud_raw);
        fpfh.SetTargetPointCloud(target_cloud_raw);
    } else {
        std::cout << "Wrong fpfh mode" << std::endl;
        return;
    }
    fpfh.Run();

    // Mismatched
    if (fpfh.GetMatchedSize() < fpfh_match_threshold) {
        std::cout << "Mismatched" << std::endl;
        return;
    } 
    // registration
    else {
        PointCloudType output_transform;
        // coarse registration
        if (registration_mode == 1) {               // Quatro
        Quatro<PointType, PointType> quatro;
        Quatro<PointType, PointType>::Params params;

        SetParams(noise_bound, noise_bound_coeff, estimating_scale, 
                num_max_iter, gnc_factor, rot_cost_diff_thr, "Quatro", params);
        quatro.reset(params);

        CloudPtr srcMatched(new pcl::PointCloud<PointType>);
        CloudPtr tgtMatched(new pcl::PointCloud<PointType>);
        *srcMatched = fpfh.GetSrcKps();
        *tgtMatched = fpfh.GetTgtKps();
        quatro.setInputSource(srcMatched);
        quatro.setInputTarget(tgtMatched);

        TicToc quatro_time;
        quatro.computeTransformation(output);

        // std::cout << "Quatro output:" << std::endl << output << std::endl;
        std::cout << "Quatro cost: " << quatro_time.Toc() << " ms" << std::endl;
        std::cout << "Quatro Finished" << std::endl;

        pcl::transformPointCloud(source_cloud_raw, output_transform, output);
        pcd_writer.writeBinary(pcd_path + "output_Quatro.pcd", output_transform);
        } else if (registration_mode == 2) {        // Teaser++
            RobustRegistrationSolver::Params params;
            SetParams(noise_bound, noise_bound_coeff, estimating_scale, 
                    num_max_iter, gnc_factor, rot_cost_diff_thr, params);
            std::vector<std::pair<int, int>> corr_ = fpfh.GetCorrespondences();
            RobustRegistrationSolver solver(params);

            PointCloudType srcMatched;
            PointCloudType tgtMatched;
            srcMatched = fpfh.GetSrcKps();
            tgtMatched = fpfh.GetTgtKps();
            PointCloud srcMatched_teaser;
            PointCloud tgtMatched_teaser;
            pcl2teaser<PointType>(srcMatched, srcMatched_teaser);
            pcl2teaser<PointType>(tgtMatched, tgtMatched_teaser);

            TicToc teaser_time;
            solver.solve(srcMatched_teaser, tgtMatched_teaser, corr_);
            auto solution = solver.getSolution();

            output.block<3, 3>(0, 0) = solution.rotation;
            output.block<3, 1>(0, 3) = solution.translation;

            // std::cout << "Estimated rotation: " << std::endl;
            // std::cout << solution.rotation << std::endl;
            // std::cout << "Estimated translation: " << std::endl;
            // std::cout << solution.translation << std::endl;
            // std::cout << "Teaser output:" << std::endl << output << std::endl;
            std::cout << "Teaser cost: " << teaser_time.Toc() << " ms" << std::endl;
            std::cout << "Teaser Finished" << std::endl;

            pcl::transformPointCloud(source_cloud_raw, output_transform, output);
            pcd_writer.writeBinary(pcd_path + "output_Teaser.pcd", output_transform);
        } else {
            std::cout << "Skip coarse registration" << std::endl;
        }

        // accurate registration
        TicToc final_fit_time;
        Fitter fitter;
        fitter.SetTargetPointCloud(target_cloud_raw);
        if (registration_mode == 1 || registration_mode == 2) {
            fitter.SetSourcePointCloud(output_transform);
            fitter.Best_Fit();
            output = fitter.GetTransformtionMatrix() * output;
            pcl::transformPointCloud(source_cloud_raw, output_transform, output);
            pcd_writer.writeBinary(pcd_path + "final_fit.pcd", output_transform);
        } else {
            fitter.SetSourcePointCloud(source_cloud_raw);
            fitter.Best_Fit();
            output = fitter.GetTransformtionMatrix();
            pcl::transformPointCloud(source_cloud_raw, output_transform, output);
            pcd_writer.writeBinary(pcd_path + "directly_accurate_fit.pcd", output_transform);
        }

        std::cout << "final output: " << std::endl << output << std::endl;
    }
}

} // namespace pcr