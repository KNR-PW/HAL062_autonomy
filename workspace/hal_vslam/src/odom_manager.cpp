#include <chrono>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <isaac_ros_visual_slam_interfaces/srv/set_slam_pose.hpp>
#include <isaac_ros_visual_slam_interfaces/srv/reset.hpp>


// Ten node jest pośrednikiem 
// SLAM -> OdomManager -> NAV2
// Jeśli chcemy zablokować Odometrię ze SLAM, to właśnie jest miejsce na to
// Za pomocą /odom_manager/switch -> TRUE == SLAM (passthrough poprzez timer) |  FALSE == Odom z memory


class OdomManager : public rclcpp::Node {
private:
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    nav_msgs::msg::Odometry current_odom_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr switch_service_;
    rclcpp::Client<isaac_ros_visual_slam_interfaces::srv::Reset>::SharedPtr slam_reset_client_;
    rclcpp::Client<isaac_ros_visual_slam_interfaces::srv::SetSlamPose>::SharedPtr slam_set_pose_client_;
    rclcpp::TimerBase::SharedPtr odmo_pub_timer_;
    std::atomic<bool> use_visual_slam_odom_{true};

public:
    OdomManager() : Node("odom_manager") {
        RCLCPP_INFO(this->get_logger(), "[OdomManager] Node started");
        client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/visual_slam/tracking/odometry", 10, std::bind(&OdomManager::odomCallback, this, std::placeholders::_1));
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_manager/odometry", 10);
        switch_service_ = this->create_service<std_srvs::srv::SetBool>("/odom_manager/switch", std::bind(&OdomManager::switchCallback, this, std::placeholders::_1, std::placeholders::_2));
        odmo_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this](){ this->odom_pub_->publish(this->current_odom_); });
        slam_reset_client_ = this->create_client<isaac_ros_visual_slam_interfaces::srv::Reset>("/visual_slam/reset", rclcpp::ServicesQoS(), client_cb_group_);
        slam_set_pose_client_ = this->create_client<isaac_ros_visual_slam_interfaces::srv::SetSlamPose>("/visual_slam/set_slam_pose", rclcpp::ServicesQoS(), client_cb_group_);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (use_visual_slam_odom_.load()) {  
            current_odom_ = *msg;
        }
    }

    void switchCallback(const std_srvs::srv::SetBool::Request::SharedPtr req, const std_srvs::srv::SetBool::Response::SharedPtr resp) {
        if (use_visual_slam_odom_.load() != static_cast<bool>(req->data)) {
            if (static_cast<bool>(req->data)){
                // TURN ON SLAM: this->bool na true + reset + set_slam_pose == current pose 
                auto reset_req = std::make_shared<isaac_ros_visual_slam_interfaces::srv::Reset::Request>();
                auto set_pose_req = std::make_shared<isaac_ros_visual_slam_interfaces::srv::SetSlamPose::Request>();
                set_pose_req->pose = current_odom_.pose.pose;
                // SLAM RESET
                auto future_reset = slam_reset_client_->async_send_request(reset_req);
                if (future_reset.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
                    RCLCPP_ERROR(get_logger(), "Timeout");
                    resp->success = false;
                    return;
                }

                // SLAM SET POSE
                auto future_set_pose = slam_set_pose_client_->async_send_request(set_pose_req);
                if (future_set_pose.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
                    RCLCPP_ERROR(get_logger(), "Timeout");
                    resp->success = false;
                    return;
                }
                use_visual_slam_odom_.store(true);
            }
            else {
                // TURN OFF SLAM: this->bool na false
                use_visual_slam_odom_.store(false);
            }
        }
        resp->success = true;
    }
};




int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomManager>();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}