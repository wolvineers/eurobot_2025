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
    string path_aida = "/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/eurobot_ws/src/tests/src/arucos_detection/assets/playmat.png";
    string path_docker = "/wolvi/src/tests/src/arucos_detection/assets/playmat.png";

    // Read the image
    Mat playmat = imread(path_docker);

    // Throw error in case the image is empty
    if (playmat.empty()) {
        cerr << "Error: Unable to open the image file!" << endl;
        return -1;
    }


    // -- ARUCO DETECTION --

    // Set the dictionary
    Ptr<Dictionary> opencvDict = getPredefinedDictionary(DICT_4X4_100);

    // Declare result variables
    vector<int> ids;
    vector<vector<Point2f>> corners;

    // Detect arucos
    detectMarkers(playmat, opencvDict, corners, ids);


    // -- ARUCO DRAWING --

    Mat resPlaymat = playmat.clone();

    // Draw all the corners
    if (!ids.empty()) {
        drawDetectedMarkers(resPlaymat, corners, ids);
        for (size_t i = 0; i < ids.size(); i++) {
            cout << "ID detected: " << ids[i] << " with vectors in the coordenates: " << endl;
            for (const auto& point : corners[i]) cout << point << endl;
        }
    } else {
        cout << "Not arucos detected." << endl;
    }

    // Show image with the results
    Mat resized_playmat;
    resize(resPlaymat, resized_playmat, Size(600, 400));
    imshow("Arucos detector", resPlaymat);
    waitKey(0);

    return 0;
}
