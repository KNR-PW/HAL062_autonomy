#include <iostream>
#include <vector>
#include <memory>
#include <cmath>
#include <map>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>


class Detector{

private:

    cv::Ptr<cv::aruco::DetectorParameters> detector_params = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary;

    cv::Mat camera_matrix = cv::Mat(3,3,cv::DataType<double>::type);
    std::vector<double> dist_coef;

    double marker_side;
    std::vector<cv::Point3f> real_points;

    std::map<int,int> marker_counter;
    
public:

    Detector(const cv::aruco::PREDEFINED_DICTIONARY_NAME& predef_dict, const double& marker_side,
            const std::vector<int> wanted_id ,const int& frame_width, const int& frame_height,
            const double& h_fov, std::vector<double> dist_coef);
    ~Detector();

    void decrementMarkerCounter();
    void incrementMarkerCounter(const int& id);
    bool checkMarkerCounter(const int& id);

    std::vector<ArucoTag> MyDetectMarkers(cv::Mat& frame);
    cv::Mat drawMarkers(cv::Mat frame, std::vector<ArucoTag> markers);
    
};

