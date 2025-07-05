#pragma once

#include "example_package/filter_node.h"
#include <eigen3/Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <example_package/KalmanFilterConfig.h>

class KalmanFilter : public FilterNode
{
public:
    KalmanFilter(ros::NodeHandle &nh);

private:
    void prediction() override;
    void correction() override;
    void reconfigCallback(example_package::KalmanFilterConfig &config, uint32_t level);

    // Kalman filter-specific members
    Eigen::MatrixXd _A = Eigen::Matrix3d::Identity(3, 3);
    Eigen::MatrixXd _B = Eigen::Matrix3d::Zero(3, 3);
    Eigen::MatrixXd _C = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Vector3d _mu_t0 = Eigen::Vector3d::Zero();
    Eigen::Matrix3d _Cov_t0 = (Eigen::Matrix3d() << 1.04490316e-05, -3.89057204e-06, 3.60840176e-05,
                               -3.89057204e-06, 5.70461249e-06, -1.53585014e-05,
                               3.60840176e-05, -1.53585014e-05, 1.75726829e-03)
                                  .finished();
    Eigen::Matrix3d _Q = (Eigen::Matrix3d() << 3.84694703e-04, 3.40030857e-05, 1.01914036e-05,
                          3.40030857e-05, 4.78394237e-04, -8.92618589e-08,
                          1.01914036e-05, -8.92618589e-08, 2.65893178e-05)
                             .finished();
    Eigen::MatrixXd _R = (Eigen::Matrix3d() << 1.04490316e-05, -3.89057204e-06, 3.60840176e-05,
                          -3.89057204e-06, 5.70461249e-06, -1.53585014e-05,
                          3.60840176e-05, -1.53585014e-05, 1.75726829e-03)
                             .finished();
    Eigen::MatrixXd _K = Eigen::Matrix3d::Zero(3, 3);
    Eigen::MatrixXd _I = Eigen::Matrix3d::Identity(3, 3);
    Eigen::Vector3d _mu_t1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d _pred_mu_t1 = Eigen::Vector3d::Zero();
    Eigen::MatrixXd _Cov_t1 = Eigen::Matrix3d::Identity(3, 3);
    Eigen::MatrixXd _pred_Cov_t1 = Eigen::Matrix3d::Identity(3, 3);

    dynamic_reconfigure::Server<example_package::KalmanFilterConfig> dr_srv_;
    dynamic_reconfigure::Server<example_package::KalmanFilterConfig>::CallbackType dr_cb_;
};