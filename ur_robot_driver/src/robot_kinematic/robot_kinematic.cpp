#include "robot_kinematic/robot_kinematic.hpp"

// Costruttore che inizializza i membri dati
RobotKinematic::RobotKinematic(const std::string& robot_description, 
                                   const std::string& root_name, 
                                   const std::string& tip_name)
    : robot_description_(robot_description), 
      root_name_(root_name), 
      tip_name_(tip_name) 
{
    if (robot_description_.empty()) {
        std::cerr << "[ERROR]: Parameter 'robot_description' is empty" << std::endl;
        std::exit(EXIT_FAILURE); // Uscita con codice di errore
    } else {
        std::cout << "[INFO]: Robot description parameter uploaded." << std::endl;
    }

    // Parse the URDF string
    if (!kdl_parser::treeFromString(robot_description_, kdl_tree_)) {
        std::cerr << "[ERROR]: Failed to parse URDF file" << std::endl;
        std::exit(EXIT_FAILURE); // Uscita con codice di errore
    } else {
        std::cout << "[INFO]: KDL Tree initialized." << std::endl;
    }

    // Ottieni i segmenti dell'albero
    const auto& tree_segments_ = kdl_tree_.getSegments();
    if (tree_segments_.empty()) {
        std::cerr << "[ERROR]: The tree is empty." << std::endl;
        std::exit(EXIT_FAILURE); // Uscita con codice di errore
    } else {
        std::cout << "kdl tree:" << std::endl;
        // If tree_segments_ is initialized correctly, then prints segment names
        for (const auto& link : tree_segments_) {
            std::cout << "[INFO]: KDL segment name: " << link.first << std::endl;
        }
    }

    // Populate the KDL chain
    if(!kdl_tree_.getChain(root_name_, tip_name_, kdl_chain_)) {
        std::cerr << "[ERROR]: Failed to get KDL chain from tree" << std::endl;
        std::exit(EXIT_FAILURE); // Uscita con codice di errore
    } else {
        std::cout << "[INFO]: KDL Chain initialized with " << kdl_chain_.getNrOfJoints() << " elements." << std::endl;    
    }

    q_measured_.resize(kdl_chain_.getNrOfJoints());
	q_.resize(kdl_chain_.getNrOfJoints());
	J_.resize(kdl_chain_.getNrOfJoints());
	q_eig_ = Eigen::VectorXd::Zero(kdl_chain_.getNrOfJoints());

	jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
	fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

    std::cout << kdl_chain_.getNrOfJoints() << std::endl;
}

// bool RobotKinematic::Init(const KDL::JntArray& q_)
// {

// }

// Funzione per calcolare la cinematica diretta
std::pair<Eigen::Vector3d, Eigen::Quaterniond> RobotKinematic::ComputeDirectKinematic(const KDL::JntArray& q_)
{
    // Esegui la cinematica diretta: da q_ a x_ (posizione e orientamento)
    fk_pos_solver_->JntToCart(q_, x_);

    // Converti la posizione da KDL a Eigen
    Eigen::Vector3d pos;
    for (int i = 0; i < 3; i++) {
        pos(i) = x_.p(i);
    }

    // Converti l'orientamento da matrice di rotazione (KDL) a quaternion (Eigen)
    Eigen::Matrix3d orient;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            orient(i, j) = x_.M(i, j);
        }
    }

    // Converti la matrice di rotazione in quaternion e normalizzalo
    Eigen::Quaterniond quat(orient);
    quat.normalize();

    // Ritorna la posizione e il quaternion
    return std::make_pair(pos, quat);
}
