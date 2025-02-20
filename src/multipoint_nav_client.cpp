#include <memory>
#include <chrono>
#include <string>
#include <iostream>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class NavigationNode : public rclcpp::Node {
public:
    NavigationNode() : Node("navigation_node") {
        // Create the action client for navigation
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        // Create publisher for visualization marker
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "goal_marker", 10);

        // Initialize initial pose publisher
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10);

        // Set initial pose after a short delay to ensure publishers are ready
        timer_ = this->create_wall_timer(
            2s, std::bind(&NavigationNode::setInitialPose, this));
    }

    void setInitialPose() {
        // Cancel the timer after first execution
        timer_->cancel();

        auto initial_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
        initial_pose.header.frame_id = "map";
        initial_pose.header.stamp = this->now();

        // Set your initial pose here
        initial_pose.pose.pose.position.x = 0.0;
        initial_pose.pose.pose.position.y = 0.0;
        initial_pose.pose.pose.position.z = 0.0;
        initial_pose.pose.pose.orientation.w = 1.0;

        // Set covariance
        for(size_t i = 0; i < 36; ++i) {
            initial_pose.pose.covariance[i] = 0.0;
        }
        initial_pose.pose.covariance[0] = 0.25;  // X variance
        initial_pose.pose.covariance[7] = 0.25;  // Y variance
        initial_pose.pose.covariance[35] = 0.06853892326654787;  // Yaw variance

        RCLCPP_INFO(this->get_logger(), "Setting initial pose...");
        initial_pose_pub_->publish(initial_pose);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // Get multiple goals from user
        getMultipleGoals();
    }

    void getMultipleGoals() {
        int num_points;
        std::cout << "Enter number of waypoints to navigate: ";
        std::cin >> num_points;
        
        if (num_points <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid number of points. Must be greater than 0.");
            getMultipleGoals();
            return;
        }
        
        waypoints_.clear();
        
        for (int i = 0; i < num_points; i++) {
            double x, y;
            std::cout << "Enter waypoint " << (i+1) << " coordinates (x y): ";
            std::cin >> x >> y;
            
            waypoints_.push_back(std::make_pair(x, y));
        }
        
        std::cout << "\nPlanned route:" << std::endl;
        for (size_t i = 0; i < waypoints_.size(); i++) {
            std::cout << "  " << (i+1) << ". (" << waypoints_[i].first << ", " 
                      << waypoints_[i].second << ")" << std::endl;
        }
        
        // Start navigation sequence
        current_waypoint_index_ = 0;
        navigateToNextWaypoint();
    }
    
    void navigateToNextWaypoint() {
        if (current_waypoint_index_ >= waypoints_.size()) {
            RCLCPP_INFO(this->get_logger(), "Navigation sequence completed! All waypoints reached.");
            std::cout << "\nNavigation sequence completed! Enter new sequence? (y/n): ";
            char response;
            std::cin >> response;
            if (response == 'y' || response == 'Y') {
                getMultipleGoals();
            }
            return;
        }
        
        auto [x, y] = waypoints_[current_waypoint_index_];
        current_goal_x_ = x;
        current_goal_y_ = y;
        
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "Navigating to waypoint %zu: x=%.2f, y=%.2f", 
                   current_waypoint_index_ + 1, x, y);
        sendGoal(goal_msg);
    }

private:
    void sendGoal(const NavigateToPose::Goal& goal) {
        if (!nav_client_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Navigation action server not available");
            return;
        }

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&NavigationNode::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&NavigationNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&NavigationNode::resultCallback, this, std::placeholders::_1);

        nav_client_->async_send_goal(goal, send_goal_options);
    }

    void goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr& goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            // Skip to next waypoint
            current_waypoint_index_++;
            navigateToNextWaypoint();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
    }

    void feedbackCallback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Waypoint %zu - Distance remaining: %.2f",
                   current_waypoint_index_ + 1, feedback->distance_remaining);
    }

    void resultCallback(const GoalHandleNavigateToPose::WrappedResult& result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Waypoint %zu reached!", current_waypoint_index_ + 1);
                publishMarker(current_goal_x_, current_goal_y_);
                // Move to next waypoint
                current_waypoint_index_++;
                navigateToNextWaypoint();
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                // Move to next waypoint
                current_waypoint_index_++;
                navigateToNextWaypoint();
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                // Move to next waypoint
                current_waypoint_index_++;
                navigateToNextWaypoint();
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                // Move to next waypoint
                current_waypoint_index_++;
                navigateToNextWaypoint();
                break;
        }
    }

    void publishMarker(double x, double y) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "goal_markers";
        marker.id = marker_count_++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.0;
        
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 1.0;
        
        // Bright blue color
        marker.color.r = 0.0f;
        marker.color.g = 0.5f;
        marker.color.b = 1.0f;
        marker.color.a = 0.8f;
        
        marker.lifetime = rclcpp::Duration::from_seconds(0);  // Persistent marker
        
        marker_pub_->publish(marker);
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double current_goal_x_ = 0.0;
    double current_goal_y_ = 0.0;
    int marker_count_ = 0;
    
    // Multi-waypoint navigation
    std::vector<std::pair<double, double>> waypoints_;
    size_t current_waypoint_index_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}