#include <iostream>
#include <memory>
#include <vector>
#include <opencv2/opencv.hpp>


class ArucoTag{

private:

    int id;
    cv::Mat rotation = cv::Mat(3,1,cv::DataType<double>::type);
    cv::Mat translation = cv::Mat(3,1,cv::DataType<double>::type);

    std::vector<cv::Point2f> visualization_corners;
    
public:

    ArucoTag();
    ArucoTag(const int& id);
    ArucoTag(const int& id, const cv::Mat& rotation, const cv::Mat& translation);
    ~ArucoTag();


    int getID();
    cv::Mat getRotation();
    cv::Mat getTranslation();
    std::vector<cv::Point2f> getCorners();


    void setTranslation(const cv::Mat& new_translation);
    void setRotation(const cv::Mat& new_rotation);
    void setCorners(const std::vector<cv::Point2f>& corners);
    
};