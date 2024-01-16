#include "base_controller_arm/ArmModel.h"

ArmModel::ArmModel(){
    // load the model from urdf file
    // pinocchio::urdf::buildModel(urdf_filename, model);
    std::cout << "model name: " << model.name << " load successfully! " << std::endl;
    data = Data(model);
    q = pinocchio::neutral(model);
    dq = Eigen::VectorXd::Zero(model.nv);
    J = Eigen::MatrixXd::Zero(6, model.nv);
    dJ = Eigen::MatrixXd::Zero(6, model.nv);

    eps = 1e-4;
    IT_MAX = 1000;
    DT = 1e-2;
    damp = 1e-6;
}

void ArmModel::InverseKinematic(Eigen::VectorXd q_now, Eigen::Vector3d Pdes, Eigen::VectorXd &qdes, Eigen::Matrix3d rot)
{
    JOINT_ID = 6;
    dq.setZero();
    q.setZero();
    q = q_now;

    pinocchio::SE3 oMdes(rot, Pdes);
    for (int i = 0;; i++)
    {
        pinocchio::forwardKinematics(model, data, q);
        const pinocchio::SE3 dMi = oMdes.actInv(data.oMi[JOINT_ID]);
        err = pinocchio::log6(dMi).toVector();
// if(i==0){
// std::cout << "err: " << err << std::endl; 
// std::cout << "init err.norm: " << err.norm() << std::endl; 
// }
        if (err.norm() < eps)
        {
            success = true;
// std::cout << "final err.norm: " << err.norm() << std::endl; 
// std::cout << "final EE pos/rpy: " << data.oMi[JOINT_ID] << std::endl;
            break;
        }
        if (i >= IT_MAX)
        {
            success = false;
            // std::cout << "conergence failed" << std::endl;
            break;
        }
        pinocchio::computeJointJacobian(model, data, q, JOINT_ID, J);
        pinocchio::Data::Matrix6 JJt;
        JJt.noalias() = J * J.transpose();
        JJt.diagonal().array() += damp;
        dq.noalias() = -J.transpose() * JJt.ldlt().solve(err);
        q = pinocchio::integrate(model, q, dq * DT);
    }

    if (success)
    {
        // for (int i = 0; i < model.nq; i++)
        // {
        //     q(i) = fmod(q(i), (2 * M_PI));
        //     if (q(i) >= M_PI && q(i) <= 2 * M_PI)
        //     {
        //         q(i) = q(i) - 2 * M_PI;
        //     }
        //     if (q(i) <= -M_PI && q(i) >= -2 * M_PI)
        //     {
        //         q(i) = q(i) + 2 * M_PI;
        //     }
        // }
        // // std::cout << "success!" << std::endl;
        qdes = q.block<6, 1>(0, 0);
    }
    else
    {
        qdes = q_now;
        // std::cout << "Joint id:" << JOINT_ID << std::endl
                //   << " Warning: the iterative algorithm has not reached convergence to the desired precision"
                //   << std::endl;
    }
}

void ArmModel::ForwardKinematic(Eigen::VectorXd q_now)
{
    q = q_now;
    pinocchio::forwardKinematics(model, data, q);
}

Eigen::Vector3d ArmModel::rot2rpy(Eigen::Matrix3d R)
{
    Eigen::Vector3d rpy;
    rpy(0) = atan2(R(2, 1), R(2, 2));
    rpy(1) = atan2(-R(2, 0), std::pow(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2), 0.5));
    rpy(2) = atan2(R(1, 0), R(0, 0));
    return rpy;
}

Eigen::Matrix3d ArmModel::rpy2rot(Eigen::Vector3d euler){
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());
    return rot;
}