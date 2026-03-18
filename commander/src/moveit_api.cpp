#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <fstream>


using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
std::ifstream file("coords.txt");
std::vector<std::vector<double>> coords_list;


class Commander {
public:
    Commander(std::shared_ptr<rclcpp::Node> node) { 
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm"); 
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);
        //goToPoseTarget(0.4, 0.0, 0.1, 0.0, 0.0, 0.0);
        //goToJointTarget({1.0,1.0,0.0});
        //goToPositionTarget(0.095, -0.02, 0.164);
        //sleep(10.0);
        /*auto pose = arm_->getCurrentPose();

        RCLCPP_INFO(node_->get_logger(),
            "End Effector Position: x=%.3f y=%.3f z=%.3f",
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z);*/

        // Timer (10 Hz)
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Commander::update, this));
        //visualise
        marker_pub_= node_->create_publisher<visualization_msgs::msg::Marker>("draw", 10);
        initMarker();
    }

    void followCartesianTrajectory(const std::vector< std::vector<double> > &coords_list, double ee_step, double jump_threshold) {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        for (std::size_t i = 0; i < coords_list.size(); i++) {
            auto coord = coords_list[i]; 
            geometry_msgs::msg::Pose target;
            target.position.x = coord[0];
            target.position.y = coord[1];
            target.position.z = coord[2];
            target.orientation.x = 0.0;
            target.orientation.y = 1.0;
            target.orientation.z = 0.0;
            target.orientation.w = 0.0;
            waypoints.push_back(target);
        }
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double fraction = arm_->computeCartesianPath(waypoints, ee_step, jump_threshold, trajectory);
        RCLCPP_INFO(node_->get_logger(), "Cartesian path computed (%.2f%%)", fraction * 100.0);
        if (fraction > 0.0) {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            arm_->execute(plan);
        }
        else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to compute Cartesian path");
        }
    }

    void printCurrentPose() {
        auto pose = arm_->getCurrentPose();
        double theta_x, theta_y, theta_z;
        tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
        tf2::Matrix3x3(q).getRPY(theta_z, theta_y, theta_x);
        RCLCPP_INFO(node_->get_logger(),
            "Endy Effector Position: x=%.3f y=%.3f z=%.3f ox=%.3f oy=%.3f oz=%.3f ow=%.3f roll=%.3f pitch=%.3f yaw=%.3f",
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
            theta_x, theta_y, theta_z
            );
    }

    void goToNamedTarget(const std::string &target) {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(target);
        planAndExecute(arm_);
    }

    void goToJointTarget(const std::vector<double> &joint_values) {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joint_values);
        planAndExecute(arm_);
    }

    void goToPoseTarget(double x, double y, double z, double ox, double oy, double oz, double ow) {          
        std::string ee_link = arm_->getEndEffectorLink();
        RCLCPP_INFO(node_->get_logger(), "End Effector Link: %s", ee_link.c_str());
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = "base_link";
        target.pose.position.x = x;
        target.pose.position.y = y;
        target.pose.position.z = z;
        target.pose.orientation.x = ox;
        target.pose.orientation.y = oy;
        target.pose.orientation.z = oz;
        target.pose.orientation.w = ow;
        arm_->setStartStateToCurrentState();
        arm_->setPoseTarget(target, ee_link);
        planAndExecute(arm_);
    }

    void goToPositionTarget(double x, double y, double z) {
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = "base_link";
        target.pose.position.x = x;
        target.pose.position.y = y;
        target.pose.position.z = z;
        arm_->setStartStateToCurrentState();
        arm_->setPositionTarget(x, y, z);
        planAndExecute(arm_);
    }

private:

    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface) {
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(node_->get_logger(), "Planning successful, executing...");
            interface->asyncExecute(plan);
        }
        else {
            RCLCPP_ERROR(node_->get_logger(), "Planning failed");
        }
    }

    void initMarker()
    {
        marker_.header.frame_id = "world";  // change to "world" if needed
        marker_.ns = "drawing";
        marker_.id = 0;
        marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker_.action = visualization_msgs::msg::Marker::ADD;

        marker_.scale.x = 0.005;  // thickness

        marker_.color.r = 1.0;
        marker_.color.g = 0.0;
        marker_.color.b = 0.0;
        marker_.color.a = 1.0;
    }

    void update()
    {
        auto pose = arm_->getCurrentPose();
        double x = pose.pose.position.x;
        double y = pose.pose.position.y;
        double z = pose.pose.position.z;

        // ---- CONTACT LOGIC ----
        double canvas_z = 0.2;
        bool touching = std::abs(z - canvas_z) < 0.005;

        if (touching)
        {
            geometry_msgs::msg::Point p;
            p.x = x;
            p.y = y;
            p.z = canvas_z;  // flatten to plane

            if (shouldAddPoint(p))
            {
                marker_.points.push_back(p);
                last_point_ = p;
                has_last_ = true;
            }
        }

        marker_.header.stamp = node_->now();
        marker_pub_->publish(marker_);
    }

    bool shouldAddPoint(const geometry_msgs::msg::Point &p)
    {
        if (!has_last_)
            return true;

        double dx = p.x - last_point_.x;
        double dy = p.y - last_point_.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        return dist > 0.002;  // spacing threshold
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    visualization_msgs::msg::Marker marker_;
    geometry_msgs::msg::Point last_point_;
    bool has_last_ = false;
};

int main (int argc, char** argv) {
    rclcpp::init(argc, argv);

    double x, y, z;
    while (file >> x >> y >> z) {
        coords_list.push_back({x, y, z});
    }
    auto node = std::make_shared<rclcpp::Node>("commander_node");
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // Create executor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // Spin in separate thread
    std::thread spinner([&executor]() {executor.spin();});
    auto commander = Commander(node);

    //commander.goToJointTarget({1.0,0.0,0.0,0.0});
    //commander.goToPositionTarget(0.55, -0.30, 0.15);
    //commander.goToPoseTarget(-0.50, -0.30, 0.15, 0.0, 1.0, 0.0, 0.0);
    
    /*for (std::size_t i = 0; i < coords_list.size(); i++) {
        auto p = coords_list[i];
        commander.goToPositionTarget(p[0], p[1], p[2]);
    }*/

    //std::this_thread::sleep_for(std::chrono::seconds(10));
    commander.followCartesianTrajectory(coords_list, 0.01, 0.0);
    commander.goToNamedTarget("home");
    commander.printCurrentPose();
    //rclcpp::spin(node);
    rclcpp::shutdown();
    spinner.join();
    return 0;
}