#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <map>

using namespace cv;
using namespace std;
using namespace aruco;


// *************************
// FUNCTIONS
// *************************

Mat parseCameraMatrix(istream& in) {
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
            cout << "VAL: " << val << endl;
            mat.at<double>(row, col++) = val;
            if (ss.peek() == ',' || ss.peek() == ';') ss.ignore();
        }
        
        row++;
    }

    return mat;
}

Mat parseDistCoeffs(istream& in) {
    /*
        Parses the distortion coefficients from the input stream and stores the values in a 1x5 matrix.
    
        Arguments:
            in (istream&): The input stream containing the distortion coefficients data.
    
        Returns:
            Mat: The 1x5 matrix containing the distortion coefficients.
    */
    
    // Create an initialize the matrix and variables
    Mat distCoeffs(1, 5, CV_64F);
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
                distCoeffs.at<double>(0, col++) = val;
                if (ss.peek() == ',' || ss.peek() == ';') ss.ignore();
            }
        }
    }

    return distCoeffs;
}



// *************************
// MAIN CODE
// *************************

int main() {
    
    // -- READING THE IMAGE -- 

    // Load test image
    Mat img = imread("/wolvi/src/computing_zone/aruco_detection/assets/playmat_02/photo_1.png");
    int h = img.rows;
    int w = img.cols;

    // Read the values of the camera calibration
    ifstream mtx_file("/wolvi/src/computing_zone/aruco_detection/calibration_values/camera_matrix_output.txt");
    Mat mtx = parseCameraMatrix(mtx_file);
    mtx_file.close();

    ifstream dits_file("/wolvi/src/computing_zone/aruco_detection/calibration_values/distortion_coefficients_output.txt");
    Mat dist = parseDistCoeffs(dits_file);
    dits_file.close();

    // Calculare the optimal camera matrix
    Mat newCameraMatrix; Rect roi;
    newCameraMatrix = getOptimalNewCameraMatrix(mtx, dist, Size(w, h), 1, Size(w, h), &roi);

    // Undistort the image
    Mat dst;
    undistort(img, dst, mtx, dist, newCameraMatrix);

    // Crop the image
    dst = dst(roi);

    if (dst.empty()) {
        cerr << "Error: Camera matrix or distortion coefficients are empty!" << endl;
        return -1;
    }


    // -- ARUCO DETECTION --

    // Set the dictionary
    Ptr<Dictionary> opencvDict = getPredefinedDictionary(DICT_4X4_100);

    // Declare result variables
    vector<int> ids;
    vector<vector<Point2f>> corners;

    // Detect arucos
    detectMarkers(dst, opencvDict, corners, ids);


    // -- ARUCO DRAWING --

    // // Draw all the corners
    // if (!ids.empty()) {
    //     drawDetectedMarkers(dst, corners, ids);
    //     for (size_t i = 0; i < ids.size(); i++) {
    //         // cout << "ID detected: " << ids[i] << " with vectors in the coordenates: " << endl;
    //         // for (const auto& point : corners[i]) cout << point << endl;
    //     }
    // } else {
    //     cout << "Not arucos detected." << endl;
    // }


    // -- ARUCO POSE ESTIMATION --

    // Variables per a la posició i orientació
    vector<Vec3d> rvecs, tvecs; // Rotació i translació

    // Estimar la posició dels marcadors
    for (size_t i = 0; i < ids.size(); i++) {
        // Estimació de la posició (rotació i translació) dels marcadors detectats
        estimatePoseSingleMarkers(corners, 0.05, mtx, dist, rvecs, tvecs);

        // Dibuixar l'eix 3D del marcador
        drawAxis(dst, mtx, dist, rvecs[i], tvecs[i], 0.1); // 0.1 és la mida dels eixos

        // Mostrar la informació de la posició i orientació
        cout << "Marker ID: " << ids[i] << endl;
        cout << "Rotation vector (rvec): " << rvecs[i] << endl;
        cout << "Translation vector (tvec): " << tvecs[i] << endl;
    }

    


    // -- PROJECTIVE TRANSFORMATION --

    // Assuming you have 4 corners of the markers in the 2D image space and 4 known points in the 3D space
    // vector<Point2f> corners2D = {corners[0][0], corners[1][0], corners[2][0], corners[3][0]};  // 2D detected points

    // cout << "Corners 2d: " << corners2D << endl;

    // // Dibuxar els punts 2D sobre la imatge
    // for (size_t i = 0; i < corners2D.size(); i++) {
    //     circle(dst, corners2D[i], 5, Scalar(0, 0, 255), -1); // Dibuxar els punts en vermell
    //     putText(dst, to_string(i + 1), corners2D[i], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2); // Afegir números al costat
    // }

    // // Define the 3D coordinates of the marker corners based on known distances
    // vector<Point3f> objectPoints = {
    //     Point3f(90, 40, 0),        // Point 1 (ArUco 1)
    //     Point3f(-90, 40, 0),      // Point 2 (ArUco 2), assuming a distance of 0.2 meters between the markers
    //     Point3f(-90, -40, 0),      // Point 3 (ArUco 3)
    //     Point3f(90, -40, 0)     // Point 4 (ArUco 4)
    // };


    // // int w1 = dst.cols;  // Ample de la imatge original
    // // int h1 = dst.rows;  // Alt de la imatge original

    // // int w2 = 300;  // Ample de la imatge de projecte
    // // int h2 = 300;  // Alt de la imatge de projecte

    // // // Vector per als punts escalats per la imatge original
    // // vector<Point2f> scaledPoints;

    // // // Calcular els factors d'escalat per x i y
    // // float scaleX = (float)w1 / w2;
    // // float scaleY = (float)h1 / h2;

    // // // Escalar els punts projectats per a que es corresponen amb la mida de la imatge original
    // // for (const auto& pt : objectPoints) {
    // //     scaledPoints.push_back(Point2f(pt.x * scaleX, pt.y * scaleY));
    // // }

    // // // Dibuixar els punts escalats sobre la imatge original
    // // for (const auto& pt : scaledPoints) {
    // //     circle(dst, pt, 5, Scalar(0, 0, 255), -1); // Punts en vermell
    // // }

    // int height = 300;
    // int width = 300;
    // Mat image = Mat::zeros(height, width, CV_8UC3);

    // // Transformar els punts 3D a 2D
    // vector<Point2f> imagePoints;
    // for (const auto& point : objectPoints) {
    //     // Projectar en 2D (simple transformació X, Y)
    //     float x = point.x + width / 2; // Desplaçar al centre de la finestra
    //     float y = -point.y + height / 2; // Invertir Y perquè el sistema de coordenades de la imatge té Y creixent cap avall

    //     imagePoints.push_back(Point2f(x, y));
    // }

    // // Dibuixar els punts a la imatge
    // for (size_t i = 0; i < imagePoints.size(); i++) {
    //     circle(image, imagePoints[i], 5, Scalar(0, 0, 255), -1); // Dibuixar els punts en vermell
    //     putText(image, to_string(i+1), imagePoints[i], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2); // Afegir números
    // }

    // // Mostrar la imatge amb els punts
    // imshow("3D Points Projected to 2D", image);

    // while (true) {
    //     char key = (char)waitKey(1);
    //     if (key == 'q' || key == 'Q') break;
    // }


    // // Show image with the results
    // imshow("Arucos detector", dst);
    
    // while (true) {
    //     char key = (char)waitKey(1);
    //     if (key == 'q' || key == 'Q') break;
    // }

    // // Now we compute the Homography matrix using the 2D and 3D points
    // Mat H = findHomography(corners2D, imagePoints);

    // // Apply the perspective warp (this is the projective transformation)
    // Mat warpedImage;
    // warpPerspective(dst, warpedImage, H, Size(300, 300));

    // // Show the image with the transformed (warped) perspective
    // imshow("Warped Image", dst);
    
    // while (true) {
    //     char key = (char)waitKey(1);
    //     if (key == 'q' || key == 'Q') break;
    // }

    destroyAllWindows();
    return 0;
}
