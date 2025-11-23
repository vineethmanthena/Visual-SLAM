#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/stereo.hpp>
#include <filesystem>
#include <vector>
#include <string>
#include <tf2/LinearMath/Quaternion.h>


namespace fs = std::filesystem;

class StereoVO : public rclcpp::Node
{
public:
    StereoVO(const std::string &left_folder, const std::string &right_folder)
        : Node("stereo_vo"), left_folder_(left_folder), right_folder_(right_folder)
    {
        // Publisher for disparity visualization
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("disparity_image", 10);
        // Publisher for trajectory
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("vo_path", 10);

        // Load camera intrinsics (KITTI)
        f_ = 718.856;
        cx_ = 607.1928;
        cy_ = 185.2157;
        b_ = 0.54; // baseline in meters

        // Initialize path message
        path_msg_.header.frame_id = "map";

        // Get sorted image lists
        for (const auto &p : fs::directory_iterator(left_folder_))
            if (p.path().extension() == ".png") left_images_.push_back(p.path().string());
        for (const auto &p : fs::directory_iterator(right_folder_))
            if (p.path().extension() == ".png") right_images_.push_back(p.path().string());

        std::sort(left_images_.begin(), left_images_.end());
        std::sort(right_images_.begin(), right_images_.end());

        // Initialize pose
        curr_pose_ = cv::Mat::eye(4, 4, CV_64F);

        // Timer for processing images sequentially
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&StereoVO::processNextFrame, this)
        );
    }

private:
    void processNextFrame()
    {
        if (idx_ >= left_images_.size() - 1) {
            RCLCPP_INFO(this->get_logger(), "Finished all images");
            rclcpp::shutdown();
            return;
        }

        cv::Mat left_img = cv::imread(left_images_[idx_], cv::IMREAD_GRAYSCALE);
        cv::Mat right_img = cv::imread(right_images_[idx_], cv::IMREAD_GRAYSCALE);
        cv::Mat next_left_img = cv::imread(left_images_[idx_ + 1], cv::IMREAD_GRAYSCALE);

        if (left_img.empty() || right_img.empty() || next_left_img.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read images at idx=%zu", idx_);
            idx_++;
            return;
        }

        // Compute disparity
        cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(
            0, 112, 5
        );
        cv::Mat disparity;
        stereo->compute(left_img, right_img, disparity);
        cv::Mat disparity_f;
        disparity.convertTo(disparity_f, CV_32F, 1.0 / 16.0);
        cv::Mat valid = disparity_f > 0.1;
        cv::Mat depths = cv::Mat::zeros(disparity.size(), CV_32F);
        cv::Mat nonzero_disp = disparity_f.clone();
        nonzero_disp.setTo(1e-6, ~valid);
        depths = (b_ * f_) / nonzero_disp;
        depths.setTo(0.0f, ~valid);

        // Detect features and descriptors
        cv::Ptr<cv::SIFT> sift = cv::SIFT::create(2000);
        std::vector<cv::KeyPoint> kpts, kpts_next;
        cv::Mat desc, desc_next;
        sift->detectAndCompute(left_img, cv::noArray(), kpts, desc);
        sift->detectAndCompute(next_left_img, cv::noArray(), kpts_next, desc_next);

        // Match features
        std::vector<std::vector<cv::DMatch>> knn_matches;
        cv::BFMatcher matcher(cv::NORM_L2);
        matcher.knnMatch(desc, desc_next, knn_matches, 2);

        std::vector<cv::DMatch> good_matches;
        for (auto &m : knn_matches) {
            if (m[0].distance < 0.75 * m[1].distance)
                good_matches.push_back(m[0]);
        }

        // Build 3D-2D correspondences
        std::vector<cv::Point3f> pts3d;
        std::vector<cv::Point2f> pts2d;
        for (auto &m : good_matches) {
            int u = cvRound(kpts[m.queryIdx].pt.x);
            int v = cvRound(kpts[m.queryIdx].pt.y);
            if (u < 0 || u >= depths.cols || v < 0 || v >= depths.rows) continue;
            float Z = depths.at<float>(v, u);
            if (Z <= 0.0f) continue;
            double X = (u - cx_) * Z / f_;
            double Y = (v - cy_) * Z / f_;
            pts3d.emplace_back(X, Y, Z);
            pts2d.push_back(kpts_next[m.trainIdx].pt);
        }

        if (pts3d.size() < 6 || pts3d.size() != pts2d.size()) {
            RCLCPP_WARN(this->get_logger(), "Not enough correspondences");
            idx_++;
            return;
        }

        // Solve PnP
        cv::Mat K = (cv::Mat_<double>(3, 3) << f_, 0, cx_, 0, f_, cy_, 0, 0, 1);
        cv::Mat rvec, tvec;
        bool ok = cv::solvePnPRansac(pts3d, pts2d, K, cv::noArray(), rvec, tvec, false, 100, 8.0, 0.99, cv::noArray(), cv::SOLVEPNP_ITERATIVE);
        if (!ok) {
            RCLCPP_WARN(this->get_logger(), "PnP failed at idx=%zu", idx_);
            idx_++;
            return;
        }

        cv::Mat R;
        cv::Rodrigues(rvec, R);
        cv::Mat Rt = cv::Mat::eye(4, 4, CV_64F);
        R.copyTo(Rt(cv::Rect(0, 0, 3, 3)));
        tvec.copyTo(Rt(cv::Rect(3, 0, 1, 3)));
        curr_pose_ = curr_pose_ * Rt.inv(); // accumulate

        // Publish path
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = this->now();
        pose_msg.pose.position.x = curr_pose_.at<double>(0, 3);
        pose_msg.pose.position.y = curr_pose_.at<double>(1, 3);
        pose_msg.pose.position.z = curr_pose_.at<double>(2, 3);
        cv::Mat R_curr = curr_pose_(cv::Rect(0, 0, 3, 3));
        // Convert rotation matrix to quaternion
        cv::Mat rvec_curr;
        cv::Rodrigues(R_curr, rvec_curr);
        tf2::Quaternion q;
        q.setRPY(rvec_curr.at<double>(0), rvec_curr.at<double>(1), rvec_curr.at<double>(2));
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
        path_msg_.poses.push_back(pose_msg);
        path_pub_->publish(path_msg_);

        // Publish disparity image
        cv::Mat disp_vis;
        cv::normalize(disparity, disp_vis, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::applyColorMap(disp_vis, disp_vis, cv::COLORMAP_JET);
        cv_bridge::CvImage img_msg;
        img_msg.encoding = sensor_msgs::image_encodings::BGR8;
        img_msg.image = disp_vis;
        image_pub_->publish(*img_msg.toImageMsg());

        idx_++;
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> left_images_;
    std::vector<std::string> right_images_;
    size_t idx_ = 0;

    std::string left_folder_, right_folder_;
    float f_, cx_, cy_, b_;
    cv::Mat curr_pose_;
    nav_msgs::msg::Path path_msg_;
};

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cout << "Usage: ros2 run vslam stereo_vo_node <left_folder> <right_folder>\n";
        return 0;
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoVO>(argv[1], argv[2]));
    rclcpp::shutdown();
    return 0;
}
