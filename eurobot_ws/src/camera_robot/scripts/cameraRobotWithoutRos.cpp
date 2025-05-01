#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <iostream>
#include <numeric>  // For std::accumulate

using namespace std;

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

int main() {
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        cerr << "Error: Could not open camera." << endl;
        return 1;
    }

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::getDefault();

    while (true) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        vector<vector<cv::Point2f>> corners;
        vector<int> ids;
        cv::aruco::detectMarkers(gray, dictionary, corners, ids, parameters);

        vector<cv::Point2f> detected_centers;

        if (!ids.empty()) {
            for (const auto& marker_corners : corners) {
                float cx = 0, cy = 0;
                for (const auto& pt : marker_corners) {
                    cx += pt.x;
                    cy += pt.y;
                }
                cx /= marker_corners.size();
                cy /= marker_corners.size();
                detected_centers.emplace_back(cx, cy);
            }

            auto groups = group_aruco_markers(detected_centers);
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
                float position_percentage = (avg_center_x / frame.cols) * 100.0f;

                string pos_str;
                if (position_percentage <= 40) pos_str = "Left";
                else if (position_percentage <= 60) pos_str = "Center";
                else pos_str = "Right";

                cout << "Detected can position: " << pos_str << endl;
            }
        }

        // Display the frame
        cv::aruco::drawDetectedMarkers(frame, corners, ids);
        cv::imshow("Aruco Detection", frame);

        if (cv::waitKey(10) == 27) break; // Press ESC to exit
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
