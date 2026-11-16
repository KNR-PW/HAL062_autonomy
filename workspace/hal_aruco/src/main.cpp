#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>
#include <map>

#include "hal_aruco/msg/aruco_pose.hpp"
#include "hal_aruco/srv/aruco_pose.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "Aruco.cpp"
#include "Detector.cpp"




static std::string FRAME_ID = "zed_left_camera_optical_frame";
static int FRAME_WIDTH = 1280;
static int FRAME_HEIGHT = 720;
static double MARKER_SIDE = 0.2;
static double HORIZONTAL_FOV = 1.9199;
static std::vector<double> DISTORTION = {-0.0435,0.0119,-0.0008,-0.0003,-0.0054};
static std::map<int,hal_aruco::msg::ArucoPose> storage;

static std::vector<int> WANTED_ID = {0,10};
static std::map<int,std::vector<float>> COLOR_ID = {
    {0,{0.0,0.7,0.5}},
    {10,{0.1,0.2,0.9}}
};




class ArucoObserver: public rclcpp::Node{

private:

    bool visualize;

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> image_subscription;
    std::shared_ptr<rclcpp::Publisher<hal_aruco::msg::ArucoPose>> pose_publisher;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> image_publisher;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> marker_publisher;
    std::shared_ptr<rclcpp::Service<hal_aruco::srv::ArucoPose>> pose_service;


    Detector detector = Detector(
        cv::aruco::DICT_5X5_250,
        MARKER_SIDE,
        WANTED_ID,
        FRAME_WIDTH,
        FRAME_HEIGHT,
        HORIZONTAL_FOV,
        DISTORTION
    );




    void imageCallback(const std::shared_ptr<sensor_msgs::msg::Image> msg){

        cv_bridge::CvImagePtr frame_ptr;

        try{
            frame_ptr = cv_bridge::toCvCopy(msg,msg->encoding);
        }
        catch(cv_bridge::Exception& error){
            RCLCPP_ERROR(this->get_logger(),"hal_aruco::imageCallback : %s",error.what());
        }
        
        detectArucoOnImage(frame_ptr->image);
    }




    cv::Mat calculateQuaterion(cv::Mat& rvecs){

        double angle = cv::norm(rvecs, cv::NORM_L2);
        cv::Mat normalized = rvecs * (1.0f/angle);
        cv::Mat quaterion = cv::Mat::zeros(1,4,CV_64F);

        quaterion.at<double>(0,0) = (rvecs.at<double>(0,0) / angle) * sin(angle / 2.0f);
        quaterion.at<double>(0,1) = (rvecs.at<double>(0,1) / angle) * sin(angle / 2.0f);
        quaterion.at<double>(0,2) = (rvecs.at<double>(0,2) / angle) * sin(angle / 2.0f);
        quaterion.at<double>(0,3) = cos(angle / 2.0f);

        return quaterion;
    }




    void detectArucoOnImage(cv::Mat& frame){

        std::vector<ArucoTag> markers = this->detector.MyDetectMarkers(frame);

        for(int i = 0; i < int(markers.size()); i++){
            this->publishArucoPose(markers[i]);
        }

        if (this->visualize){
            this->publishImage(frame, markers);
            this->publishMarker(markers);
        }

    }




    void publishArucoPose(ArucoTag marker){

        auto msg = hal_aruco::msg::ArucoPose();

        int marker_id = marker.getID();
        cv::Mat translation = marker.getTranslation();
        cv::Mat rotation = marker.getRotation();


        msg.id = marker_id;
        msg.pose_stamped.pose.position.x = translation.at<double>(0,0);
        msg.pose_stamped.pose.position.y = translation.at<double>(0,1);
        msg.pose_stamped.pose.position.z = translation.at<double>(0,2);

        cv::Mat quaterion = this->calculateQuaterion(rotation);

        msg.pose_stamped.pose.orientation.x = quaterion.at<double>(0,0);
        msg.pose_stamped.pose.orientation.y = quaterion.at<double>(0,1);
        msg.pose_stamped.pose.orientation.z = quaterion.at<double>(0,2);
        msg.pose_stamped.pose.orientation.w = quaterion.at<double>(0,3);

        msg.pose_stamped.header.frame_id = FRAME_ID;
        msg.pose_stamped.header.stamp = this->get_clock()->now();

        storage.insert_or_assign(marker_id,msg);


        this->pose_publisher->publish(msg);

    }




    void publishImage(cv::Mat image, std::vector<ArucoTag> markers){

        if (int(markers.size()) != 0){
            cv::Mat visualization =  this->detector.drawMarkers(image, markers);
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8", visualization).toImageMsg();
            auto ros_msg = *msg.get();
            ros_msg.header.frame_id = FRAME_ID;
            this->image_publisher->publish(ros_msg);
        }

        else{
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(),"bgra8", image).toImageMsg();
            auto ros_msg = *msg.get();
            ros_msg.header.frame_id = FRAME_ID;
            ros_msg.header.stamp = this->get_clock()->now();
            this->image_publisher->publish(ros_msg);
        }
    }



    void publishMarker(std::vector<ArucoTag> markers){
        int id_offset = 10000;
        for(int i = 0; i < int(markers.size()); i++){

            int marker_id = markers[i].getID();
            cv::Mat translation = markers[i].getTranslation();
            cv::Mat rotation = markers[i].getRotation();
            std::vector<float> color = COLOR_ID[marker_id];

            auto marker = visualization_msgs::msg::Marker();
            auto text = visualization_msgs::msg::Marker();

            marker.header.frame_id = FRAME_ID;
            marker.header.stamp = this->get_clock()->now();
            marker.id = id_offset + marker_id;

            text.header.frame_id = FRAME_ID;
            text.header.stamp = this->get_clock()->now();
            text.id = -1 * (id_offset + marker_id);

            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = 0;

            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = 0;
            text.text = std::to_string(marker_id);

            marker.pose.position.x = translation.at<double>(0,0);
            marker.pose.position.y = translation.at<double>(0,1);
            marker.pose.position.z = translation.at<double>(0,2);

            text.pose.position.x = translation.at<double>(0,0);
            text.pose.position.y = translation.at<double>(0,1) - MARKER_SIDE;
            text.pose.position.z = translation.at<double>(0,2);

            cv::Mat quaterion = this->calculateQuaterion(rotation);

            marker.pose.orientation.x = quaterion.at<double>(0,0);
            marker.pose.orientation.y = quaterion.at<double>(0,1);
            marker.pose.orientation.z = quaterion.at<double>(0,2);
            marker.pose.orientation.w = quaterion.at<double>(0,3);

            text.pose.orientation.x = quaterion.at<double>(0,0);
            text.pose.orientation.y = quaterion.at<double>(0,1);
            text.pose.orientation.z = quaterion.at<double>(0,2);
            text.pose.orientation.w = quaterion.at<double>(0,3);

            marker.scale.x = MARKER_SIDE;
            marker.scale.y = MARKER_SIDE;
            marker.scale.z = MARKER_SIDE;

            text.scale.x = MARKER_SIDE;
            text.scale.y = MARKER_SIDE;
            text.scale.z = MARKER_SIDE;

            marker.color.a = 1.0;
            marker.color.r = color[0];
            marker.color.g = color[1];
            marker.color.b = color[2];

            text.color.a = 1.0;
            text.color.r = color[0];
            text.color.g = color[1];
            text.color.b = color[2];

            this->marker_publisher->publish(marker);
            this->marker_publisher->publish(text);

        }
        
    }




    void callbackArucoPose(const std::shared_ptr<hal_aruco::srv::ArucoPose::Request> request,
                           const std::shared_ptr<hal_aruco::srv::ArucoPose::Response> response){
        if(storage.find(request->id) != storage.end()){
            response->aruco_pose = storage[request->id];
        }
        else{
            auto empty = hal_aruco::msg::ArucoPose();
            empty.id = -1;
            response->aruco_pose = empty;
        }

    }



public:
    ArucoObserver() : rclcpp::Node("aruco_node"){

        this->image_subscription = this->create_subscription<sensor_msgs::msg::Image>("/zed/zed_node/left/color/rect/image",
                                        rclcpp::SensorDataQoS(),std::bind(&ArucoObserver::imageCallback,this,std::placeholders::_1));

        this->pose_publisher = this->create_publisher<hal_aruco::msg::ArucoPose>("/hal_aruco/aruco_pose",10);

        this->image_publisher = this->create_publisher<sensor_msgs::msg::Image>("/hal_aruco/image",10);

        this->marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>("/hal_aruco/markers",10);

        this->pose_service = this->create_service<hal_aruco::srv::ArucoPose>("/hal_aruco/get_pose",
                            std::bind(&ArucoObserver::callbackArucoPose, this, std::placeholders::_1, std::placeholders::_2));

        this->declare_parameter("visualize",true);
        rclcpp::Parameter choice = this->get_parameter("visualize");
        this->visualize = choice.as_bool();

    }
};




int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ArucoObserver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
