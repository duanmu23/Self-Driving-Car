#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/video.hpp"
#include "opencv2/videostab.hpp"

#include <sstream>
#include <iostream>
#include <fstream>
using namespace std;
using namespace cv;

const float calibrationSquareDimension = 0.01905f; // meters
const float arucoSquareDimension = 0.1016f; //meters
const Size chessboardDimension = Size(6, 9);

bool loadCameraCalibration(string name, Mat& cameraMatrix, Mat& distortionCoefficients)
{
	ifstream inStream(name);
	if (inStream)
	{
		uint16_t rows;
		uint16_t columns;

		inStream >> rows;
		inStream >> columns;

		cameraMatrix = Mat(Size(columns, rows), CV_64F);

		for (int r = 0;r < rows; r++)
		{
			for (int c = 0; c < columns;c++)
			{
				double read = 0.0f;
				inStream >> read;
				cameraMatrix.at<double>(r, c) = read;
				cout << cameraMatrix.at<double>(r, c) << "\n";

			}
		}

		inStream >> rows;
		inStream >> columns;

		distortionCoefficients = Mat::zeros(rows, columns, CV_64F);
		for (int r = 0;r < rows; r++)
		{
			for (int c = 0; c < columns;c++)
			{
				double read = 0.0f;
				inStream >> read;
				distortionCoefficients.at<double>(r, c) = read;
				cout << distortionCoefficients.at<double>(r, c) << "\n";
			}
		}

		inStream.close();
		return true;
	}
	return false;
}

//use 
void MarkerPositionFinder(Mat frame, const Mat& camerMatrix, const Mat& distortionCoefficients, float arucoSquareDimensions, Point2i& Car_position)
{
	// create variable to store the camera information
	char font = FONT_HERSHEY_PLAIN;
	string poisition;
	vector<int> markerIds;
	vector<vector<Point2f>> markerCorners, rejectedCandidates;
	aruco::DetectorParameters parameters;

	Ptr< aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
	
	vector<Vec3d> rotationVecotrs, translationVectors;
		//Detect the markers
	aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
		//Estimate the position of the markers
	aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, camerMatrix, distortionCoefficients, rotationVecotrs, translationVectors);


		for (int i = 0; i < markerIds.size();i++)
		{
			aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
			aruco::drawAxis(frame, camerMatrix, distortionCoefficients, rotationVecotrs[i], translationVectors[i], 0.1f);
			Car_position.x = translationVectors[i][0] * 746.45 - 1.1826;
			Car_position.y = translationVectors[i][1] * 647.12 - 2.0599;

		}

		//switch (character)   //used for save image, no need to use now.
		//{
		//case ' ':
		//	imwrite("E:\\UoA\\Project\\ArUco\\Aruco_code\\calibration1.jpg", frame);
		//}
}

// Read the Text file and extract the coordinate
void inputPath(string name, vector<Point2i>& road) {
	road.clear();
	
	ifstream path;
	path.open(name);
	
	int rows;       //the first line in txt indicates the rows of coordinates and columns of coordinates
	int columns;

	path >> rows;
	path >> columns;
	Point2i temp;   // temporary value to keep the position

	for (int r = 0; r < rows;r++) {
		path >> temp.x;
		path >> temp.y;
		road.push_back(temp);   //Store the position into a vector
 	}

}

void KalmanFilter() {

}

int main(int argv, char** argc) {
	vector<Point2i> track;    //The track coordinates is stored in the track
	inputPath("E:\\UoA\\Project\\ArUco\\LaneFinding\\circle_path.txt", track);
	
	Mat frame;
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	Mat distortionCoefficients;
	Point2i Car_position;

	VideoCapture vid(0);
	namedWindow("Came", cv::WINDOW_FULLSCREEN);
	vid.set(CAP_PROP_GIGA_FRAME_SENS_WIDTH, 1280);
    vid.set(CAP_PROP_GIGA_FRAME_SENS_HEIGH, 720);
	if (!vid.isOpened())
	{
		return -1;
	}
	while (true)
	{
		if (!vid.read(frame))
			break;
		MarkerPositionFinder(frame, cameraMatrix, distortionCoefficients, 0.1135f, Car_position);
		cout << Car_position << endl;
		namedWindow("Camera", cv::WINDOW_FULLSCREEN);
		imshow("Camera", frame);
		if (waitKey(30) >= 0) break;
	}

	return 0;
}