#include "../include/hal_aruco/Aruco.h"

ArucoTag::ArucoTag(){;}
ArucoTag::ArucoTag(const int& id){
    this->id = id;    
}
ArucoTag::ArucoTag(const int& id, const cv::Mat& rotation, const cv::Mat& translation){
    this->id = id;
    this->rotation = rotation;
    this->translation = translation;
}
ArucoTag::~ArucoTag(){;}




int ArucoTag::getID() {return this->id;}
cv::Mat ArucoTag::getRotation() {return this->rotation;}
cv::Mat ArucoTag::getTranslation() {return this->translation;}
std::vector<cv::Point2f> ArucoTag::getCorners() {return this->visualization_corners;}




void ArucoTag::setRotation(const cv::Mat& new_rotation){
    this->rotation = new_rotation;
}




void ArucoTag::setTranslation(const cv::Mat& new_translation){
    this->translation = new_translation;
}




void ArucoTag::setCorners(const std::vector<cv::Point2f>& corners){
    this->visualization_corners = corners;
}