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
    string path_docker = "/wolvi/src/computing_zone/aruco_detection/assets/photo2.png";

    // Read the image
    Mat playmat = imread(path_docker);

    // Read the video
    VideoCapture cap(path_docker);

    // Throw error in case the image is empty
    if (!cap.isOpened()) {
        cerr << "Error: Unable to open the video!" << endl;
        return -1;
    }


    // -- ARUCO DETECTION --

    // Set the dictionary
    Ptr<Dictionary> opencvDict = getPredefinedDictionary(DICT_4X4_100);

    // Detect the arucos in every frame
    Mat frame;

    while (true) {
        // Check if the video has ended
        if (!cap.read(frame)) {
            cerr << "End of video." << endl;
            break;
        }

        // Declare result variables
        vector<int> ids;
        vector<vector<Point2f>> corners;

        // Detect arucos
        detectMarkers(frame, opencvDict, corners, ids);


        // -- ARUCO DRAWING --

        // Draw all the corners
        if (!ids.empty()) {
            drawDetectedMarkers(frame, corners, ids);
            for (size_t i = 0; i < ids.size(); i++) {
                // cout << "ID detected: " << ids[i] << " with vectors in the coordenates: " << endl;
                // for (const auto& point : corners[i]) cout << point << endl;
            }
        } else {
            cout << "Not arucos detected." << endl;
        }

        // Show image with the results
        imshow("Arucos detector", frame);
        if (waitKey(20) == 'q') break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}
