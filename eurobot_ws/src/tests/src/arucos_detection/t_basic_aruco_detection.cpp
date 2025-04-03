#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <string>
#include <map>

using namespace cv;
using namespace std;
using namespace aruco;

int main() {
    
    // -- READING THE IMAGE -- 

    // Save the possible paths of the image
    string path_aida = "/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/eurobot_ws/src/tests/src/arucos_detection/assets/photo_8.png";
    string path_docker = "/wolvi/src/tests/src/arucos_detection/assets/photo_1.png";

    // Read the image
    Mat playmat = imread(path_docker);

    // Throw error in case the image is empty
    if (playmat.empty()) {
        cerr << "Error: Unable to open the image file!" << endl;
        return -1;
    }

    // -- CAMERA CALIBRATION MATRIX --

    // Camera matrix (3x3)
    Mat cameraMatrix = (Mat_<double>(3, 3) << 892.0741812, 0, 826.51502968,
                                             0, 881.01427636, 554.99886337,
                                             0, 0, 1);

    // Distortion coefficients (5 elements)
    Mat distCoeffs = (Mat_<double>(1, 5) << -0.34043579, 0.14535852, 0.00743814, -0.00275169, -0.03219138);


    // -- UNDISTORT THE IMAGE --

    // Apply undistort to the image
    Mat undistortedImage;
    undistort(playmat, undistortedImage, cameraMatrix, distCoeffs);


    // -- IMPROVE IMAGE CONTRAST AND REMOVE NOISE --

    // Convert the image to grayscale
    Mat grayImage;
    cvtColor(undistortedImage, grayImage, COLOR_BGR2GRAY);

    // Apply histogram equalization to improve contrast
    Mat enhancedImage;
    equalizeHist(grayImage, enhancedImage);

    // Apply Gaussian blur to reduce noise
    Mat blurredImage;
    GaussianBlur(enhancedImage, blurredImage, Size(5, 5), 0);


    // -- ARUCO DETECTION --

    // Set the dictionary
    Ptr<Dictionary> opencvDict = getPredefinedDictionary(DICT_4X4_100);

    // Declare result variables
    vector<int> ids;
    vector<vector<Point2f>> corners;

    // Detect arucos
    detectMarkers(blurredImage, opencvDict, corners, ids);


    // -- ARUCO DRAWING --

    Mat resPlaymat = undistortedImage.clone();

    // Draw all the corners
    if (!ids.empty()) {
        drawDetectedMarkers(resPlaymat, corners, ids);
        for (size_t i = 0; i < ids.size(); i++) {
            cout << "ID detected: " << ids[i] << " with vectors in the coordinates: " << endl;
            for (const auto& point : corners[i]) cout << point << endl;
        }
    } else {
        cout << "No arucos detected." << endl;
    }

    // Show image with the results
    Mat resized_playmat;
    resize(resPlaymat, resized_playmat, Size(600, 400));
    imshow("Arucos detector", resized_playmat);
    waitKey(0);

    return 0;
}
