#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class DetectFeatures final : public rclcpp::Node
{
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    std::string detector_type_;
    cv::Ptr<cv::Feature2D> detector_;

public:
    DetectFeatures() : Node("detect_features") {
        detector_type_ = declare_parameter<std::string>("detector_type", "ORB");
        create_detector(detector_type_);

        if (detector_) {
            image_sub_ = create_subscription<sensor_msgs::msg::Image>( "/image_raw", 10,
                [this](const sensor_msgs::msg::Image::SharedPtr msg) -> void { process_image(msg); });

            image_pub_ = create_publisher<sensor_msgs::msg::Image>( "/image_annotated", 10);
        }
    }

    void create_detector(const std::string& detector_type)
    {
        if (detector_type == "SIFT") {
            detector_ = cv::SIFT::create();
        } else if (detector_type == "BRISK") {
            detector_ = cv::BRISK::create();
        } else if (detector_type == "ORB") {
            detector_ = cv::ORB::create(5000);  // Default is 500, let's find a bunch
        } else if (detector_type == "AKAZE") {
            detector_ = cv::AKAZE::create();
        } else if (detector_type == "MSER") {
            detector_ = cv::MSER::create();
        } else if (detector_type == "FAST") {
            detector_ = cv::FastFeatureDetector::create();
        } else if (detector_type == "SimpleBlobDetector" || detector_type == "blob") {
            detector_ = cv::SimpleBlobDetector::create();
        } else if (detector_type == "AgastFeatureDetector" || detector_type == "Agast") {
            detector_ = cv::AgastFeatureDetector::create();
        } else if (detector_type == "GFTTDetector" || detector_type == "GFTT") {
            detector_ = cv::GFTTDetector::create();
        } else {
            RCLCPP_ERROR(get_logger(), "Unknown detector type: %s", detector_type.c_str());
        }
    }

    void process_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg) const
    {
        // Convert to cv::Mat
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg);
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Detect features
        std::vector<cv::KeyPoint> keypoints;
        detector_->detect(cv_ptr->image, keypoints);
        RCLCPP_INFO(get_logger(), "Detected %ld %s features", keypoints.size(), detector_type_.c_str());

        // Draw features
        cv::Mat annotated_image;
        drawKeypoints(cv_ptr->image, keypoints, annotated_image, cv::Scalar::all(-1),
            cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        // Publish annotated image
        const auto annotated_msg = cv_bridge::CvImage(msg->header, "bgr8", annotated_image).toImageMsg();
        image_pub_->publish(*annotated_msg);
    }

};

int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    spin(std::make_shared<DetectFeatures>());
    rclcpp::shutdown();
    return 0;
}