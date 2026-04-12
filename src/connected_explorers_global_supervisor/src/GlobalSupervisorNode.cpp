/*******************************************************************************
* Description:

- GlobalSupervisorNode

This Node is respomsable for keeping the enviroment infomartion and simulate 
plausible communication between the robots



*******************************************************************************/



/*******************************************************************************
* Includes
*******************************************************************************/

#include "rclcpp/rclcpp.hpp"

// messages
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"

// custom messages
#include "connected_explorers_interfaces/msg/sync_state.hpp"

// custom files
#include "connected_explorers_utils/MultiRobotsPoseHandler.hpp"
#include "connected_explorers_utils/ConnWeightHandler.hpp"
#include "connected_explorers_utils/LaplacianMatrixHandler.hpp"
#include "connected_explorers_utils/MapHandler.hpp"

/*******************************************************************************
* Defines
*******************************************************************************/

#define QOS_STD_PROFILE 10

#define FIEDLER_GRADIENT_TOPIC_NAME "lambda2_gradient"
#define LAPLACIAN_MATRIX_PUBLISHER_PERIOD_MS 100
#define FIEDLER_GRADIENT_PUBLISHER_PERIOD_MS 100

/*******************************************************************************
* Class definition and parameters
*******************************************************************************/

class GlobalSupervisorNode : public rclcpp::Node
{

// parameters ---
private:
    int number_of_robots_;
    std::string robot_name_prefix_;

// publishers ---
private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr laplacian_matrix_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fiedler_value_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr fiedler_gradient_publisher_;

// subscribers ---
private:

// timers ---
private:
    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr laplacian_matrix_publisher_timer_;
    rclcpp::TimerBase::SharedPtr fiedler_gradient_publisher_timer_;

// helpers ---
private:    
    std::unique_ptr<connected_explorers_utils::MultiRobotsPoseHandler> pose_handler_;
    std::unique_ptr<connected_explorers_utils::MapHandler> map_handler_;
    std::unique_ptr<connected_explorers_utils::ConnWeightHandler> conn_handler_;
    std::unique_ptr<connected_explorers_utils::LaplacianMatrixHandler> laplacian_matrix_handler_;

// data ---
private:

// mutex ---
private:
    


/*******************************************************************************
* Class constructor
*******************************************************************************/

public:
    GlobalSupervisorNode() : Node("global_supervisor")
    {

        // node parameters ---
        std::string node_param_name;

        node_param_name = "number_of_robots";
        this->declare_parameter<int>(node_param_name, 1);
        number_of_robots_ = this->get_parameter(node_param_name).as_int();

        node_param_name = "robot_name_prefix";
        this->declare_parameter<std::string>(node_param_name, "robot_");
        robot_name_prefix_ = this->get_parameter(node_param_name).as_string();
        






        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(0), 
            std::bind(&GlobalSupervisorNode::init, this)
        );
    }

/*******************************************************************************
* Class methods
*******************************************************************************/
private:
    void init() {
        init_timer_->cancel();



        laplacian_matrix_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>("laplacian_matrix", QOS_STD_PROFILE);
        fiedler_value_publisher_ = create_publisher<std_msgs::msg::Float64>("fiedler_value", QOS_STD_PROFILE);
        fiedler_gradient_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>("lambda2_gradient", QOS_STD_PROFILE);


        pose_handler_ = std::make_unique<connected_explorers_utils::MultiRobotsPoseHandler>(
            this->shared_from_this(),
            number_of_robots_,
            robot_name_prefix_,
            "/position",
            QOS_STD_PROFILE
        );

        pose_handler_->InitPoseSubscribers();


        map_handler_ = std::make_unique<connected_explorers_utils::MapHandler>(
            this->shared_from_this(),
            "/map",
            QOS_STD_PROFILE
        );

        map_handler_->InitMapSubscriber();

        conn_handler_ = std::make_unique<connected_explorers_utils::ConnWeightHandler>(
            1.0f,
            6.0f,
            -6.0f,
            0.5f
        );
        laplacian_matrix_handler_ = std::make_unique<connected_explorers_utils::LaplacianMatrixHandler>(
            number_of_robots_
        );

        StartLaplacianMatrixPublisher();
        StartFiedlerGradientPublisher();

        
        RCLCPP_INFO(this->get_logger(), "All systems initialized.");
    }

    void StartLaplacianMatrixPublisher() {
        laplacian_matrix_publisher_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(LAPLACIAN_MATRIX_PUBLISHER_PERIOD_MS), 
            std::bind(&GlobalSupervisorNode::OnLaplacianTimerTick, this));
    }

    void StartFiedlerGradientPublisher(){
        fiedler_gradient_publisher_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(FIEDLER_GRADIENT_PUBLISHER_PERIOD_MS), 
            std::bind(&GlobalSupervisorNode::PublishFiedlerGradient, this));
    }


    void PublishFiedlerGradient(){
        Eigen::VectorXd gradient = GetGradientVectorNumericWay(0.5);
        auto msg = std_msgs::msg::Float64MultiArray();

        // 1. Setup Rows (N_robots * 2)
        std_msgs::msg::MultiArrayDimension dim_rows;
        dim_rows.label = "rows";
        dim_rows.size = gradient.size();
        dim_rows.stride = gradient.size() * 1; // rows * cols
        msg.layout.dim.push_back(dim_rows);

        // 2. Setup Cols (Always 1 for a vector to keep it 2D)
        std_msgs::msg::MultiArrayDimension dim_cols;
        dim_cols.label = "cols";
        dim_cols.size = 1;
        dim_cols.stride = 1;
        msg.layout.dim.push_back(dim_cols);

        // 3. Assign data
        msg.data.assign(gradient.data(), gradient.data() + gradient.size());

        fiedler_gradient_publisher_->publish(msg);
    }


    void OnLaplacianTimerTick() {
        std::vector<geometry_msgs::msg::Pose> poses = pose_handler_->GetRobotsPoses();
        if (poses.size() < 2) {
            RCLCPP_INFO(this->get_logger(), "poses.size() < 2");
            return;
        }
        // for (geometry_msgs::msg::Pose pose:poses){
        //     RCLCPP_INFO(this->get_logger(),"x: %f | y: %f",pose.position.x,pose.position.y);
        // }
        UpdateLaplacianWeights(poses);
        PublishLaplacianData();
    }

    void UpdateLaplacianWeights(const std::vector<geometry_msgs::msg::Pose>& poses) {
        for (size_t i = 0; i < poses.size() - 1; ++i) {
            for (size_t j = i + 1; j < poses.size(); ++j) {
                auto line = map_handler_->GetLineBetweenPoints(
                    poses[i].position.x, poses[i].position.y, 
                    poses[j].position.x, poses[j].position.y);

                auto line_res = map_handler_->GetLineMinDistToObstacle(line);

                float weight = conn_handler_->CalculateFinalScore(
                    poses[i].position, poses[j].position, line_res.min_dist);
                
                laplacian_matrix_handler_->UpdateConnWeight(weight, i, j);
            }
        }
    }

    void PublishLaplacianData() {
        laplacian_matrix_publisher_->publish(laplacian_matrix_handler_->GetLaplacianMsg());

        std_msgs::msg::Float64 fiedler_msg;
        fiedler_msg.data = laplacian_matrix_handler_->GetFiedlerValue();
        fiedler_value_publisher_->publish(fiedler_msg);
    }


    double GenerateNumericLaplacianDerivative(const std::string& axis, int index, double dt) {
        // 1. Setup local copy of the adjacency matrix
        // Assuming matrix_handler_ returns an Eigen::MatrixXd
        Eigen::MatrixXd new_adj = laplacian_matrix_handler_->GetAdjacencyMatrix();
        auto poses = pose_handler_->GetRobotsPoses(); // geometry_msgs::msg::Pose

        // 2. Perturb the target robot's position
        geometry_msgs::msg::Point p1 = poses[index].position;
        if (axis == "x") p1.x += dt;
        else if (axis == "y") p1.y += dt;

        // 3. Update only the edges connected to the perturbed robot
        for (int i = 0; i < number_of_robots_; ++i) {
            if (i == index) continue;

            geometry_msgs::msg::Point p2 = poses[i].position;
            
            // Use your connection handler from the previous step
            auto line = map_handler_->GetLineBetweenPoints(
                p1.x, p1.y, 
                p2.x, p2.y
            );

            auto line_res = map_handler_->GetLineMinDistToObstacle(line);
            double score = conn_handler_->CalculateFinalScore(p1, p2, line_res.min_dist);
            
            new_adj(index, i) = score;
            new_adj(i, index) = score;
        }

        // 4. Generate Laplacian: L = D - A
        Eigen::VectorXd degrees = new_adj.rowwise().sum();
        Eigen::MatrixXd new_laplacian = degrees.asDiagonal();
        new_laplacian -= new_adj;

        // 5. Compute Eigenvalues
        // SelfAdjointEigenSolver is faster and more stable for symmetric matrices (Laplacians)
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(new_laplacian);
        if (es.info() != Eigen::Success) return 0.0;

        double new_lambda_2 = es.eigenvalues()(1); // Second smallest eigenvalue
        double old_lambda_2 = laplacian_matrix_handler_->GetFiedlerValue();

        return (new_lambda_2 - old_lambda_2) / dt;
    }

    Eigen::VectorXd GetGradientVectorNumericWay(double dt) {
    Eigen::VectorXd gradient_vector = Eigen::VectorXd::Zero(number_of_robots_ * 2);
    int counter = 0;

    for (int r_index = 0; r_index < number_of_robots_; ++r_index) {
        gradient_vector(counter++) = GenerateNumericLaplacianDerivative("x", r_index, dt);
        gradient_vector(counter++) = GenerateNumericLaplacianDerivative("y", r_index, dt);
    }
    return gradient_vector;
}




};


/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalSupervisorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}