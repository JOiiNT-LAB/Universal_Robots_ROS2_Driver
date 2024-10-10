#ifndef ROBOT_KINEMATIC_HPP
#define ROBOT_KINEMATIC_HPP

#include <iostream>
#include <string>
#include <memory>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <Eigen/Dense>

class RobotKinematic {
private:
    std::string robot_description_;
    std::string root_name_;
    std::string tip_name_;

    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;

    KDL::JntArray  q_;              // Joint computed positions
    KDL::Jacobian  J_;              // Jacobian
    KDL::Frame     x_;              // Tip pose                                                                                                                                       
    Eigen::VectorXd q_eig_;         

public:

    KDL::JntArray  q_measured_;          // Joint measured positions

    // Costruttore che inizializza i membri dati
    RobotKinematic(const std::string& robot_description, 
                   const std::string& root_name, 
                   const std::string& tip_name);

    KDL::JntArray Init(const KDL::JntArray);

    // Metodo per calcolare la cinematica diretta
    std::pair<Eigen::Vector3d, Eigen::Quaterniond> ComputeDirectKinematic(const KDL::JntArray& q_);

    // Metodi getter per accedere ai membri privati
    std::string getRobotDescription() const;
    std::string getRootName() const;
    std::string getTipName() const;

    // Metodi setter per modificare i membri privati
    void setRobotDescription(const std::string& robot_description);
    void setRootName(const std::string& root_name);
    void setTipName(const std::string& tip_name);

    // Metodo per stampare le informazioni
    void printInfo() const;
};

#endif // ROBOT_KINEMATIC_HPP
