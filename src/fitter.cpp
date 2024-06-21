#include "fitter.h"

namespace pcr {

Fitter::Fitter() {}

Fitter::~Fitter() {}

void Fitter::Best_Fit(void) {
    pcr::TicToc run_time;
    run_time.Tic();

    Eigen::Vector3d centroid_A(0,0,0); 
    Eigen::Vector3d centroid_B(0,0,0);
    Eigen::MatrixXd AA = source_cloud_matrix;
    Eigen::MatrixXd BB = target_cloud_matrix;
    int row = source_cloud_matrix.rows();

    // 计算质心
    for(int i=0; i<row; i++){
        centroid_A += source_cloud_matrix.block<1,3>(i,0).transpose();
        centroid_B += target_cloud_matrix.block<1,3>(i,0).transpose();
    }

    centroid_A /= row;
    centroid_B /= row;

    // 去中心化
    for(int i=0; i<row; i++){
        AA.block<1,3>(i,0) = source_cloud_matrix.block<1,3>(i,0) - centroid_A.transpose();
        BB.block<1,3>(i,0) = target_cloud_matrix.block<1,3>(i,0) - centroid_B.transpose();
    }

    // 计算协方差矩阵
    Eigen::MatrixXd H = AA.transpose()*BB;

    Eigen::MatrixXd U;
    Eigen::VectorXd S;
    Eigen::MatrixXd V;
    Eigen::MatrixXd Vt;

    // 进行奇异值分解 (SVD)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = svd.matrixU();
    S = svd.singularValues();
    V = svd.matrixV();
    Vt = V.transpose();

    // 计算旋转矩阵
    rotation_matrix = Vt.transpose()*U.transpose();

    // 确保旋转矩阵的行列式为正
    if (rotation_matrix.determinant() < 0 ){
        Vt.block<1,3>(2,0) *= -1;
        rotation_matrix = Vt.transpose()*U.transpose();
    }

    // 计算平移向量
    translation_vector = centroid_B - rotation_matrix*centroid_A;

    transform_matrix.block<3,3>(0,0) = rotation_matrix;
    transform_matrix.block<3,1>(0,3) = translation_vector;

    std::cout << std::endl;
    // std::cout << "accurate registration output:" << std::endl << transform_matrix << std::endl;
    std::cout << "accurate registration cost: " << run_time.Toc() << " ms" << std::endl;
    std::cout << "accurate registration Finished" << std::endl << std::endl;
}

} // namespace pcr