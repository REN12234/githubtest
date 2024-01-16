/**
 * @file ArmModel.h
 * @author Zhicheng Song
 * @brief 
 * @version 0.1
 * @date 2023-03-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef ARM_MODEL_H
#define ARM_MODEL_H

#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/centroidal-derivatives.hpp>

#include "ParamHandler.hpp"
#include "enumClass.h"
#include <Eigen/Dense>

using namespace pinocchio;

class ArmModel
{
public:
    ArmModel();
    ~ArmModel() = default;

    // set desired pos
    void InverseKinematic(Eigen::VectorXd q_now, Eigen::Vector3d Pdes, Eigen::VectorXd &qdes, Eigen::Matrix3d rot);
    void ForwardKinematic(Eigen::VectorXd q);

    Data data;
    Model model;
    Eigen::Vector3d rot2rpy(Eigen::Matrix3d R);
    Eigen::Matrix3d rpy2rot(Eigen::Vector3d euler);

private:
    Eigen::VectorXd q;
    Eigen::VectorXd dq;
    // Eigen::VectorXd v;
    Eigen::MatrixXd J;
    Eigen::MatrixXd dJ;
    Eigen::Matrix<double, 6, 1> err;

    int JOINT_ID;
    double eps;
    double damp;
    int IT_MAX;
    double DT;
    double ydes;
    bool success = false;
    // const std::string urdf_filename = std::string("../robotmodel/robotArm.urdf");
};

#endif