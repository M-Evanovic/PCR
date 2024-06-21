#pragma once

#include <iostream>
#include <stdexcept>

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types_conversion.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <cstdlib>
#include <ctime>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <flann/flann.hpp>

#include <teaser/geometry.h>

// using PointType = pcl::PointXYZINormal;
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;

using NormalPointType = pcl::Normal;
using NormalCloudType = pcl::PointCloud<NormalPointType>;
using NormalCloudPtr = NormalCloudType::Ptr;

using FPFHType = pcl::FPFHSignature33;
using FPFHCloudType = pcl::PointCloud<FPFHType>;
using FPFHCloudPtr = FPFHCloudType::Ptr;

namespace pcr {
template<typename T>
void pcl2teaser(const pcl::PointCloud<T> &pcl_raw, pcr::PointCloud &cloud) {
    cloud.clear();
    for (const auto &pt: pcl_raw.points) {
        cloud.push_back({pt.x, pt.y, pt.z});
    }
}

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "map") {
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

template<typename T>
void pcl2eigen(const pcl::PointCloud<T> &pcl_raw, Eigen::Matrix<double, 3, Eigen::Dynamic> &cloud) {
    int N = pcl_raw.points.size();
    cloud.resize(3, N);
    for (int i = 0; i < N; ++i) {
        cloud.col(i) << pcl_raw.points[i].x, pcl_raw.points[i].y, pcl_raw.points[i].z;
    }
}

template<typename T>
void Pcl2Eigen(const pcl::PointCloud<T> &pcl_raw, Eigen::MatrixXd &cloud) {
    int point_num = pcl_raw.points.size();
    cloud.resize(point_num, 3);
    for (int i = 0; i < point_num; ++i) {
        cloud.row(i) << pcl_raw.points[i].x, pcl_raw.points[i].y, pcl_raw.points[i].z;
    }
}

template<typename T>
void eigen2pcl(const Eigen::Matrix<double, 3, Eigen::Dynamic> &src, pcl::PointCloud<T> &cloud) {
    int num_pc = src.cols();
    T   pt_tmp;
    if (!cloud.empty()) cloud.clear();
    for (int i = 0; i < num_pc; ++i) {
        pt_tmp.x = src(0, i);
        pt_tmp.y = src(1, i);
        pt_tmp.z = src(2, i);
        cloud.points.emplace_back(pt_tmp);
    }
}

template <typename T>
void Eigen2Pcl(const Eigen::Matrix<double, 3, Eigen::Dynamic> &src, pcl::PointCloud<T> &cloud) {
    int num_pc = src.cols();
    T pt_tmp;
    if (!cloud.empty())
        cloud.clear();
    for (int i = 0; i < num_pc; ++i) {
        pt_tmp.x = src(0, i);
        pt_tmp.y = src(1, i);
        pt_tmp.z = src(2, i);
        cloud.points.emplace_back(pt_tmp);
    }
}

template <typename T>
void Voxelize(typename pcl::PointCloud<T>::Ptr input_cloud, 
              typename pcl::PointCloud<T>::Ptr output_cloud,
              double voxel_size) {
    pcl::VoxelGrid<T> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_filter.filter(*output_cloud);
}

class TicToc {
   public:
    TicToc() { Tic(); }

    void Tic() { start_ = std::chrono::system_clock::now(); }

    double Toc() {
        end_ = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end_ - start_;
        return elapsed_seconds.count() * 1000;
    }

   private:
    std::chrono::time_point<std::chrono::system_clock> start_, end_;
};

} // namespace pcr
