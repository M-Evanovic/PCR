#include <ros/ros.h>
#include <pcl/common/transforms.h>
// #include <nano_gicp/point_type_nano_gicp.hpp>
// #include <nano_gicp/nano_gicp.hpp>

#include "utility.h"
#include "register.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "pcr");

    ros::NodeHandle nh;

    std::string pcd_path;
    nh.param<std::string>("pcd_path", pcd_path, "./");
    std::string source_pcd_path = pcd_path + "source.pcd";
    std::string target_pcd_path = pcd_path + "target.pcd";

    PointCloudType source_cloud;
    PointCloudType target_cloud;

    // load source pointcloud
    if (pcl::io::loadPCDFile(source_pcd_path, source_cloud) == -1) {
        PCL_ERROR("couldn't load source point cloud \n");
    }

    // load target pointcloud(transform form source)
    std::vector<double> trans_vector(16, 0.0);
    nh.param<std::vector<double>>("transform", trans_vector, std::vector<double>());
    Eigen::Matrix4d true_transform;
    true_transform << trans_vector[0],  trans_vector[1],  trans_vector[2],  trans_vector[3],
                      trans_vector[4],  trans_vector[5],  trans_vector[6],  trans_vector[7],
                      trans_vector[8],  trans_vector[9],  trans_vector[10], trans_vector[11],
                      trans_vector[12], trans_vector[13], trans_vector[14], trans_vector[15];
    std::cout << "Truth:" << std::endl << true_transform << std::endl << std::endl;
    pcl::transformPointCloud(source_cloud, target_cloud, true_transform);
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(pcd_path + "true_transform.pcd", target_cloud);

    // load target pointcloud(local)
    // if (pcl::io::loadPCDFile(target_pcd_path, target_cloud) == -1) {
    //     PCL_ERROR("couldn't load target point cloud \n");
    // }

    // test1 my register
    pcr::Register _register;
    pcr::TicToc segregate_ground_time;
    _register.SetSourcePointCloud(source_cloud);
    _register.SetTargetPointCloud(target_cloud);
    std::cout << "Segregate Ground cost: " << segregate_ground_time.Toc() << " ms" << std::endl;
    _register.Run();

    // test2 gicp
    // nano_gicp::NanoGICP<PointType, PointType> m_nano_gicp;

    ////// nano_gicp init
    // m_nano_gicp.setMaxCorrespondenceDistance(max_corres_dist_);
    // m_nano_gicp.setNumThreads(thread_number_);
    // m_nano_gicp.setCorrespondenceRandomness(correspondences_number_);
    // m_nano_gicp.setMaximumIterations(100);    // max_iter_
    // m_nano_gicp.setTransformationEpsilon(transformation_epsilon_);
    // m_nano_gicp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);
    // m_nano_gicp.setRANSACIterations(ransac_max_iter_);
    // m_nano_gicp.setRANSACOutlierRejectionThreshold(ransac_outlier_rejection_threshold_);

    ////// use
    // CloudPtr src_(new pcl::PointCloud<pcl::PointXYZI>);
    // CloudPtr dst_(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PointCloud<PointType> dummy_;
    /* watch out! */
    // *src_ = source_cloud; //put your data here
    // *dst_ = target_cloud; //put your data here
    /* watch out! */
    // pcr::TicToc gicp_time;
    // m_nano_gicp.setInputSource(src_);
    // m_nano_gicp.calculateSourceCovariances();
    // m_nano_gicp.setInputTarget(dst_);
    // m_nano_gicp.calculateTargetCovariances();
    // m_nano_gicp.align(dummy_);

    // double score_ = m_nano_gicp.getFitnessScore();
    // std::cout << "score: " << score_ << std::endl;
    // if matchness score is lower than threshold, (lower is better)
    // if(m_nano_gicp.hasConverged() && score_ < icp_score_threshold) {
    // Eigen::Matrix4f pose_betweenf_ = m_nano_gicp.getFinalTransformation(); //float
    // Eigen::Matrix4d pose_betweend_ = m_nano_gicp.getFinalTransformation().cast<double>(); //double
    // std::cout << "output: " << std::endl << pose_betweend_ << std::endl;
    // }

    // std::cout << "gicp run time: " << gicp_time.Toc() << "ms" << std::endl;

    // PointCloudType result_cloud;
    // pcl::transformPointCloud(source_cloud, result_cloud, pose_betweend_);
    // pcd_writer.writeBinary(pcd_path + "gicp.pcd", result_cloud);
}
