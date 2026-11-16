#include "../include/hal_aruco/Detector.h"
#include <rclcpp/rclcpp.hpp>


Detector::Detector(const cv::aruco::PREDEFINED_DICTIONARY_NAME& predef_dict, const double& marker_side,
const std::vector<int> wanted_id, const int& frame_width, const int& frame_height,
const double& h_fov, std::vector<double> dist_coef){

    this->dictionary = cv::aruco::getPredefinedDictionary(predef_dict);
    this->marker_side = marker_side;
    this->dist_coef = dist_coef;
    double center_x = frame_width/2;
    double center_y = frame_height/2;
    double focal_length = 0.5 * frame_width / tan(0.5*h_fov);


    this->camera_matrix.at<double>(0,0) = focal_length;
    this->camera_matrix.at<double>(1,1) = focal_length;
    this->camera_matrix.at<double>(2,2) = 1.0;
    this->camera_matrix.at<double>(0,2) = center_x;
    this->camera_matrix.at<double>(1,2) = center_y;
    this->camera_matrix.at<double>(0,1) = 0.0;
    this->camera_matrix.at<double>(1,0) = 0.0;
    this->camera_matrix.at<double>(2,0) = 0.0;
    this->camera_matrix.at<double>(2,1) = 0.0;


    this->real_points = {
        cv::Point3f(-1*marker_side/2, marker_side/2, 0),
        cv::Point3f(marker_side/2, marker_side/2, 0),
        cv::Point3f(marker_side/2, -1*marker_side/2, 0),
        cv::Point3f(-1*marker_side/2, -1*marker_side/2, 0)
    };

    for(const int id: wanted_id){
        this->marker_counter.insert({id,0});
    }

}




Detector::~Detector() {;}




void Detector::decrementMarkerCounter(){
    for(auto& pair : this->marker_counter){

        if(this->marker_counter[pair.first] > 0){
            this->marker_counter[pair.first] = this->marker_counter[pair.first] - 1; 
        }

    }
}




void Detector::incrementMarkerCounter(const int& id){
    if(this->marker_counter.find(id) != this->marker_counter.end()){

        if(this->marker_counter[id] <= 20){

            this->marker_counter[id] = this->marker_counter[id] + 2;

        }
    }
}




bool Detector::checkMarkerCounter(const int& id){
    if(this->marker_counter.find(id) != this->marker_counter.end()){

        if(this->marker_counter[id] >= 7){

            return true;
        }
    }

    return false;

}




std::vector<ArucoTag> Detector::MyDetectMarkers(cv::Mat& frame){

    cv::Mat gray;
    cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);
    
    std::vector<ArucoTag> found_markers; 
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;

    cv::aruco::detectMarkers(gray, this->dictionary, corners, ids, this->detector_params, rejected);
    this->decrementMarkerCounter();

    for(int i = 0; i < int(ids.size());i++){

        this->incrementMarkerCounter(ids[i]);
        if(this->checkMarkerCounter(ids[i])){

            cv::Mat rvec(3,1,cv::DataType<double>::type);
            cv::Mat tvec(3,1,cv::DataType<double>::type);

            ArucoTag found_marker = ArucoTag(ids.at(i));
            cv::solvePnP(this->real_points, corners.at(i),this->camera_matrix,this->dist_coef,rvec,tvec);

            found_marker.setCorners(corners.at(i));
            found_marker.setRotation(rvec);
            found_marker.setTranslation(tvec);
            found_markers.push_back(found_marker);
        }
    }

    return found_markers;

}




cv::Mat Detector::drawMarkers(cv::Mat frame_copy, std::vector<ArucoTag> markers){

    std::vector<std::vector<cv::Point2f>> every_corner;
    std::vector<int> every_id;
    cv::Mat bgr_frame;

    cv::cvtColor(frame_copy,bgr_frame,cv::COLOR_BGRA2BGR);

    if(markers.size() != 0){

        for(int i = 0; i < int(markers.size()); i++){

            every_corner.push_back(markers[i].getCorners());
            every_id.push_back(markers[i].getID());

        }

        cv::aruco::drawDetectedMarkers(bgr_frame, every_corner, every_id, cv::Scalar(100,50,250));

        for(int i = 0; i < int(every_id.size()); i++){

            cv::drawFrameAxes(bgr_frame,this->camera_matrix,this->dist_coef,markers[i].getRotation(),markers[i].getTranslation(),this->marker_side*1.0f,2);

        }
        return bgr_frame;
    }
    return bgr_frame;

}
