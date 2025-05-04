#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <thread>
#include <map>

using namespace cv;
using namespace std;
using namespace aruco;
using namespace std::chrono;



// *************************
// FUNCTIONS
// *************************

Mat parse_camera_matrix(istream& in) {
    /*
        Parses the camera matrix from the input stream and stores the values in a 3x3 matrix.
    
        Arguments:
            in (istream&): The input stream containing the camera matrix data.
    
        Returns:
            Mat: The 3x3 matrix containing the camera matrix values.
    */

    // Create an initialize the matrix and variables
    Mat mat(3, 3, CV_64F);
    string line;
    int row = 0;
    bool started = false;

    // Read the stream line by line
    while (getline(in, line)) {

        // Skip lines until we find the beginning of the matrix (indicated by a '[' character)
        if (!started && line.find('[') == string::npos) continue;
        started = true;

        // Remove brackets if present
        line.erase(remove(line.begin(), line.end(), '['), line.end());
        line.erase(remove(line.begin(), line.end(), ']'), line.end());

        // Use stringstream to extract numerical values from the cleaned line
        stringstream ss(line);
        double val;
        int col = 0;

        // Read values separated by spaces, commas, or semicolons
        while (ss >> val) {
            mat.at<double>(row, col++) = val;
            if (ss.peek() == ',' || ss.peek() == ';') ss.ignore();
        }
        
        row++;
    }

    return mat;
}

Mat parse_dist_coeffs(istream& in) {
    /*
        Parses the distortion coefficients from the input stream and stores the values in a 1x5 matrix.
    
        Arguments:
            in (istream&): The input stream containing the distortion coefficients data.
    
        Returns:
            Mat: The 1x5 matrix containing the distortion coefficients.
    */
    
    // Create an initialize the matrix and variables
    Mat dist_coeffs(1, 5, CV_64F);
    string line;

    // Read the stream line by line
    while (getline(in, line)) {

        // Find the positions of the opening and closing brackets
        size_t start = line.find('[');
        size_t end = line.find(']');

        // If both brackets are found in the line, extract the data between them
        if (start != string::npos && end != string::npos) {

            // Extract substring between brackets
            line = line.substr(start + 1, end - start - 1);
            stringstream ss(line);

            double val;
            int col = 0;

            // Read each number and assign it to the matrix
            while (ss >> val) {
                dist_coeffs.at<double>(0, col++) = val;
                if (ss.peek() == ',' || ss.peek() == ';') ss.ignore();
            }
        }
    }

    return dist_coeffs;
}



// *************************
// MAIN CODE
// *************************

int main() {

    // -- PREPARE CALIBRATION VALUES --

    ifstream mtx_file("/wolvi/src/computing_zone/aruco_detection/calibration_values/camera_matrix_output.txt");
    Mat mtx = parse_camera_matrix(mtx_file);
    mtx_file.close();

    ifstream dits_file("/wolvi/src/computing_zone/aruco_detection/calibration_values/distortion_coefficients_output.txt");
    Mat dist = parse_dist_coeffs(dits_file);
    dits_file.close();


    // -- OPEN THE VIDEO --

    string video_path = "/wolvi/src/computing_zone/aruco_detection/assets/robot_demo/prova.avi";
    VideoCapture cap(video_path);

    if (!cap.isOpened()) {
        cerr << "Could not open video: " << video_path << endl;
        return -1;
    }

    // Compute the number of frames
    int total_frames = static_cast<int>(cap.get(CAP_PROP_FRAME_COUNT));
    if (total_frames <= 0) {
        cerr << "Could not get the number of frames" << endl;
        return -1;
    }

    // Desired video duration
    double desired_duration_sec = 45.0;

    // Delay per frame (ms)
    double delay_per_frame_ms = (desired_duration_sec / total_frames) * 1000.0;
    int delay = static_cast<int>(delay_per_frame_ms);


    // -- ARUCO BOARD DETECTION --

    // Set the dictionary
    Ptr<Dictionary> opencv_dict = getPredefinedDictionary(DICT_4X4_100);

    // Declare result variables
    Mat frame;
    vector<int> ids;
    vector<vector<Point2f>> corners;
    map<int, Point2f> marker_centers;

    // Arucos board
    set<int> target_ids = {20, 21, 22, 23};
    set<int> detected_ids;

    // Read frames until detect the four arucos in the board
    while (cap.read(frame) && !frame.empty()) {
        ids.clear();
        corners.clear();
        marker_centers.clear();

        // Detect arucos
        detectMarkers(frame, opencv_dict, corners, ids);

        // Calculate the corners center
        for (size_t i = 0; i < ids.size(); i++) {
            Point2f center(0, 0);
            for (const auto& point : corners[i]) center += point;
            center *= 0.25f;
            marker_centers[ids[i]] = center;

            // If the aruco is one of the wanted, save it
            if (target_ids.count(ids[i])) {
                detected_ids.insert(ids[i]);
            }
        }

        // Check if all arucos have found
        if (detected_ids.size() == target_ids.size()) {
            cout << "All arucos detected." << endl;
            break;
        }
    }



    // -- CAMERA POSE TRANSFORMATION --

    // Store the world points
    map<int, cv::Point3f> board_points_mm = {
        {20, {2400, 600, 0}},
        {21, {600, 600, 0}},
        {22, {2400, 1400, 0}},
        {23, {600, 1400, 0}}
    };

    // Store the image points from the centers calculated before
    set<int> reference_ids = {20, 21, 22, 23};

    vector<cv::Point2f> image_points;
    vector<cv::Point3f> board_points;

    for (const auto& [id, center] : marker_centers) {
        if (reference_ids.count(id) && board_points_mm.count(id)) {
            image_points.push_back(center);
            board_points.push_back(board_points_mm[id]);
        }
    }

    // Compute the camera position
    Mat board_rvec, board_tvec;
    solvePnP(board_points, image_points, mtx, dist, board_rvec, board_tvec);

    Mat board_rot;
    Rodrigues(board_rvec, board_rot);
    Mat board_rot_t = board_rot.t();

    Mat board_trans = (Mat_<double>(3,1) << board_tvec.at<double>(0), board_tvec.at<double>(1), board_tvec.at<double>(2));
    Mat camera_pos = -board_rot_t * board_trans;


    while (true) {
        if (!cap.read(frame) || frame.empty()) {
            cout << "Video ended." << endl;
            break;
        }

        // -- ARUCO DETECTION --

        // Set the dictionary
        Ptr<Dictionary> opencv_dict = getPredefinedDictionary(DICT_4X4_100);

        // Declare result variables
        vector<int> ids;
        vector<vector<Point2f>> corners;

        // Detect arucos
        detectMarkers(frame, opencv_dict, corners, ids);

        // Calculate the corners center
        std::map<int, cv::Point2f> marker_centers;

        for (size_t i = 0; i < ids.size(); i++) {
            Point2f center(0, 0);
            for (const auto& point : corners[i]) { center += point; }  // Sum all the corners
            center *= 0.25f;  // Corners average

            // Store the center with its ID
            marker_centers[ids[i]] = center;
        }

        if (!ids.empty()) drawDetectedMarkers(frame, corners, ids);


        // -- ROBOT POSE TRANSFORMATION --

        // Store the aruco points
        float marker_length = 100.0;
        vector<Point3f> robot_obj_points = {
            {-marker_length/2,  marker_length/2, 0},
            { marker_length/2,  marker_length/2, 0},
            { marker_length/2, -marker_length/2, 0},
            {-marker_length/2, -marker_length/2, 0}
        };

        set<int> reference_robot_ids = {6};

        vector<cv::Point2f> image_points_robot;
        bool found_id = false;

        for (size_t i = 0; i < ids.size(); ++i) {
            if (ids[i] == 3) {
                image_points_robot = corners[i];
                found_id = true;
                break;
            }
        }

        // Compute the aruco position
        if (found_id) {
            Mat robot_rvec, robot_tvec;
            solvePnP(robot_obj_points, image_points_robot, mtx, dist, robot_rvec, robot_tvec);
            
            Mat robot_rot;
            Rodrigues(robot_rvec, robot_rot);

            Mat robot_trans = (Mat_<double>(3,1) << robot_tvec.at<double>(0), robot_tvec.at<double>(1), robot_tvec.at<double>(2));
            Mat p_board = board_rot_t * robot_trans + camera_pos;

            double robot_x = p_board.at<double>(0);
            double robot_y = p_board.at<double>(1);
            double robot_z = p_board.at<double>(2);
            cout << "Robot position (mm): X=" << robot_x << ", Y=" << robot_y << ", Z=" << robot_z << endl;
        }

        imshow("Video reproduction", frame);

        // Wait the computed delay
        if (waitKey(delay) == 'q') break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}