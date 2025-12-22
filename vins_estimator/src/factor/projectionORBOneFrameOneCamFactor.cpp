/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "projectionORBOneFrameOneCamFactor.h"

Eigen::Matrix2d projectionORBOneFrameOneCamFactor::sqrt_info;
double projectionORBOneFrameOneCamFactor::sum_t;

projectionORBOneFrameOneCamFactor::projectionORBOneFrameOneCamFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_i_world) : 
                                       pts_i(_pts_i), pts_i_world(_pts_i_world)
{
};

bool projectionORBOneFrameOneCamFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    TicToc tic_toc;
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d tic(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond qic(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    const Eigen::Vector3d& pts_imu_i = Qi.inverse()*(pts_i_world - Pi);
    const Eigen::Vector3d& pts_camera_i = qic.inverse()*(pts_imu_i - tic);

    Eigen::Map<Eigen::Vector2d> residual(residuals);
    double dep_i = pts_camera_i.z();
    double dep_i_2 = dep_i*dep_i;
    residual = (pts_camera_i / dep_i).head<2>() - pts_i.head<2>();

    residual = sqrt_info * residual;
    // residual = -residual;
    if (jacobians)
    {

        Eigen::Matrix3d RiInv = Qi.inverse().toRotationMatrix();
        // Eigen::Matrix3d ric = qic.toRotationMatrix();
        const Eigen::Matrix3d& ricInv = qic.inverse().toRotationMatrix();

        Eigen::Matrix<double, 2, 3> reduce(2, 3);
        reduce << 1. / dep_i, 0, -pts_camera_i(0)/dep_i_2,
                0,  1. / dep_i, -pts_camera_i(1)/dep_i_2;
        reduce = sqrt_info * reduce;

        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Eigen::Matrix<double, 3, 6> jaco_i;
            jaco_i.leftCols<3>() = - ricInv * RiInv;
            jaco_i.rightCols<3>() = ricInv * Utility::skewSymmetric(pts_imu_i);
            jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
            jacobian_pose_i.rightCols<1>().setZero();
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[1]);
            Eigen::Matrix<double, 3, 6> jaco_ex;
            jaco_ex.leftCols<3>() = -ricInv;
            jaco_ex.rightCols<3>() = Utility::skewSymmetric(pts_camera_i);
            

            jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }
    }
    sum_t += tic_toc.toc();

    return true;
}