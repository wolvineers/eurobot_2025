#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <vector>
#include <set>
#include <map>
#include <cmath>
#include <numeric>
#include <chrono>
#include <algorithm>

using namespace std::chrono;
using std::placeholders::_1;

using std::vector;
using std::pair;
using std::set;

using namespace std;

class ArucoCenterPublisher : public rclcpp::Node {
public:
    ArucoCenterPublisher()
    : Node("aruco_center_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("/object_center", 10);

        cap_.open(0, cv::CAP_V4L2);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Error: No se puede abrir la cámara.");
            rclcpp::shutdown();
            return;
        }

        aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        detector_params_ = cv::aruco::DetectorParameters::create();
        detector_ = std::make_unique<cv::aruco::ArucoDetector>(aruco_dict_, detector_params_);

        buffer_start_time_ = steady_clock::now();

        timer_ = this->create_wall_timer(
            10ms, std::bind(&ArucoCenterPublisher::process_frame, this));
    }

    ~ArucoCenterPublisher() {
        cap_.release();
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;

    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    std::unique_ptr<cv::aruco::ArucoDetector> detector_;

    vector<int> position_buffer_;
    steady_clock::time_point buffer_start_time_;

    double euclidean_distance(const cv::Point2f& p1, const cv::Point2f& p2) {
        return cv::norm(p1 - p2);
    }

    vector<vector<cv::Point2f>> group_aruco_markers(const vector<cv::Point2f>& centers, float max_distance = 150.0) {
        vector<vector<cv::Point2f>> groups;
        set<int> visited;

        for (size_t i = 0; i < centers.size(); ++i) {
            if (visited.count(i)) continue;
            vector<cv::Point2f> group = { centers[i] };
            visited.insert(i);

            for (size_t j = 0; j < centers.size(); ++j) {
                if (!visited.count(j) && euclidean_distance(centers[i], centers[j]) < max_distance) {
                    group.push_back(centers[j]);
                    visited.insert(j);
                }
            }

            groups.push_back(group);
        }

        return groups;
    }

    void process_frame() {
        cv::Mat frame;
        if (!cap_.read(frame)) {
            RCLCPP_ERROR(this->get_logger(), "Error: No se pudo capturar la imagen.");
            return;
        }

        int width = frame.cols;

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        vector<int> ids;
        vector<vector<cv::Point2f>> corners;
        detector_->detectMarkers(gray, corners, ids);

        vector<cv::Point2f> detected_centers;

        if (!ids.empty()) {
            double max_area = 0;

            for (const auto& marker_corners : corners) {
                float cx = 0, cy = 0;
                for (const auto& pt : marker_corners) {
                    cx += pt.x;
                    cy += pt.y;
                }
                cx /= marker_corners.size();
                cy /= marker_corners.size();
                detected_centers.emplace_back(cx, cy);

                double area = std::fabs(cv::contourArea(marker_corners));
                if (area > max_area) {
                    max_area = area;
                }
            }

            auto groups = group_aruco_markers(detected_centers, 150.0f);
            vector<vector<cv::Point2f>> cans;

            for (const auto& group : groups) {
                if (group.size() == 1 || group.size() == 2) {
                    cans.push_back(group);
                }
            }

            if (!cans.empty()) {
                vector<float> all_centers_x;
                for (const auto& group : cans) {
                    float sum_x = 0;
                    for (const auto& p : group) sum_x += p.x;
                    all_centers_x.push_back(sum_x / group.size());
                }

                float avg_center_x = std::accumulate(all_centers_x.begin(), all_centers_x.end(), 0.0f) / all_centers_x.size();
                float position_percentage = (avg_center_x / width) * 100.0f;

                int position = 1; // Center by default
                if (position_percentage <= 40) position = 0;
                else if (position_percentage > 60) position = 2;

                position_buffer_.push_back(position);
            }
        }

        auto now = steady_clock::now();
        auto elapsed = duration_cast<milliseconds>(now - buffer_start_time_).count();

        if (elapsed >= 100) {
            if (!position_buffer_.empty()) {
                std::map<int, int> counts;
                for (int pos : position_buffer_) ++counts[pos];

                int most_common = std::max_element(
                    counts.begin(), counts.end(),
                    [](const auto& a, const auto& b) { return a.second < b.second; }
                )->first;

                std_msgs::msg::Int32 msg;
                msg.data = most_common;
                publisher_->publish(msg);

                RCLCPP_INFO(this->get_logger(), "Publicado: %d", most_common);
            }

            position_buffer_.clear();
            buffer_start_time_ = now;
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoCenterPublisher>());
    rclcpp::shutdown();
    return 0;
}
