#include "stdafx.h"
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;
using namespace std;

bool calibrate(Mat& intrMat, Mat& distCoeffs, vector<vector<Point2f>>& imagePoints,
	vector<vector<Point3f>>& ObjectPoints, Size& imageSize, const int cameraId,
	vector<string> imageList);
static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners);
vector<string> readStringList();

int rec()
{
	//initialize some parameters
	bool okcalib = false;
	Mat intrMatFirst, intrMatSec, distCoeffsFirst, distCoffesSec;
	Mat R, T, E, F, RFirst, RSec, PFirst, PSec, Q;
	vector<vector<Point2f>> imagePointsFirst, imagePointsSec;
	vector<vector<Point3f>> ObjectPoints(1);
	Rect validRoi[2];
	Size imageSize;
	int cameraIdFirst = 0, cameraIdSec = 1;
	double rms = 0;

	//get pictures and calibrate
	vector<string> imageList;
	string filename = "stereo_calib.xml";
	bool okread = true;//readStringList(filename, imageList);

	imageList = readStringList();

	if (!okread || imageList.empty())
	{
		cout << "can not open " << filename << " or the string list is empty" << endl;
		return false;
	}
	if (imageList.size() % 2 != 0)
	{
		cout << "Error: the image list contains odd (non-even) number of elements\n";
		return false;
	}

	//calibrate
	cout << "calibrate left camera..." << endl;
	okcalib = calibrate(intrMatFirst, distCoeffsFirst, imagePointsFirst, ObjectPoints,
		imageSize, cameraIdFirst, imageList);

	if (!okcalib)
	{
		cout << "fail to calibrate left camera" << endl;
		return -1;
	}
	else
	{
		cout << "calibrate the right camera..." << endl;
	}

	okcalib = calibrate(intrMatSec, distCoffesSec, imagePointsSec, ObjectPoints,
		imageSize, cameraIdSec, imageList);

	if (!okcalib)
	{
		cout << "fail to calibrate the right camera" << endl;
		return -1;
	}
	destroyAllWindows();

	//estimate position and orientation
	cout << "estimate position and orientation of the second camera" << endl
		<< "relative to the first camera..." << endl;
	rms = stereoCalibrate(ObjectPoints, imagePointsFirst, imagePointsSec,
		intrMatFirst, distCoeffsFirst, intrMatSec, distCoffesSec,
		imageSize, R, T, E, F,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 1e-6),CV_CALIB_FIX_INTRINSIC);
	cout << "done with RMS error=" << rms << endl;

	//stereo rectify
	cout << "stereo rectify..." << endl;
	stereoRectify(intrMatFirst, distCoeffsFirst, intrMatSec, distCoffesSec, imageSize, R, T, RFirst,
		RSec, PFirst, PSec, Q, 0, 1, imageSize, &validRoi[0], &validRoi[1]);

	//read pictures for 3d-reconstruction
	namedWindow("canvas", 0);
	cout << "read the picture for 3d-reconstruction...";
	Mat canvas(imageSize.height, imageSize.width * 2, CV_8UC3), viewLeft, viewRight;
	Mat canLeft = canvas(Rect(0, 0, imageSize.width, imageSize.height));
	Mat canRight = canvas(Rect(imageSize.width, 0, imageSize.width, imageSize.height));
	viewLeft = imread(imageList[cameraIdFirst], 1);
	viewRight = imread(imageList[cameraIdSec], 1);
	viewLeft.copyTo(canLeft);
	viewRight.copyTo(canRight);
	cout << "done" << endl;
	imshow("canvas", canvas);
	waitKey(50);

	//stereoRectify
	Mat rmapFirst[2], rmapSec[2], rviewFirst, rviewSec;
	initUndistortRectifyMap(intrMatFirst, distCoeffsFirst, RFirst, PFirst,
		imageSize, CV_16SC2, rmapFirst[0], rmapFirst[1]);
	initUndistortRectifyMap(intrMatSec, distCoffesSec, RSec, PSec,
		imageSize, CV_16SC2, rmapSec[0], rmapSec[1]);
	remap(viewLeft, rviewFirst, rmapFirst[0], rmapFirst[1], INTER_LINEAR);
	remap(viewRight, rviewSec, rmapSec[0], rmapSec[1], INTER_LINEAR);
	rviewFirst.copyTo(canLeft);
	rviewSec.copyTo(canRight);

	rectangle(canLeft, validRoi[0], Scalar(255, 0, 0), 3, 8);
	rectangle(canRight, validRoi[1], Scalar(255, 0, 0), 3, 8);
	for (int j = 0; j <= canvas.rows; j += 16)
		line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
	cout << "stereo rectify done" << endl;
	imshow("canvas", canvas);
	waitKey(50);
	system("pause");
}


vector<string> readStringList()
{
	vector<string> res;
	res.push_back("l1.jpg");
	res.push_back("r1.jpg");
	res.push_back("l2.jpg");
	res.push_back("r2.jpg");
	res.push_back("l3.jpg");
	res.push_back("r3.jpg");
	res.push_back("l4.jpg");
	res.push_back("r4.jpg");
	res.push_back("l5.jpg");
	res.push_back("r5.jpg");

	return res;
}


bool calibrate(Mat& intrMat, Mat& distCoeffs, vector<vector<Point2f>>& imagePoints,
	vector<vector<Point3f>>& ObjectPoints, Size& imageSize, const int cameraId,
	vector<string> imageList)
{
	int w = 6;
	int h = 9;
	double rms = 0;

	Size boardSize;
	boardSize.width = w;
	boardSize.height = h;
	vector<Point2f> pointBuf;
	float squareSize = 1.f;
	vector<Mat> rvecs, tvecs;
	bool ok = false;

	int nImages = (int)imageList.size() / 2;
	namedWindow("View", 1);
	for (int i = 0; i<nImages; i++)
	{
		Mat view, viewGray;
		view = imread(imageList[i * 2 + cameraId], 1);
		imageSize = view.size();
		cvtColor(view, viewGray, COLOR_BGR2GRAY);

		bool found = findChessboardCorners(view, boardSize, pointBuf,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

		if (found)
		{
			cornerSubPix(viewGray, pointBuf, Size(11, 11),
				Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(view, boardSize, Mat(pointBuf), found);
			bitwise_not(view, view);
			imagePoints.push_back(pointBuf);
			cout << '.';
		}
		imshow("View", view);
		waitKey(50);
	}
	//calculate chessboardCorners
	calcChessboardCorners(boardSize, squareSize, ObjectPoints[0]);
	ObjectPoints.resize(imagePoints.size(), ObjectPoints[0]);

	rms = calibrateCamera(ObjectPoints, imagePoints, imageSize, intrMat, distCoeffs,
		rvecs, tvecs);
	ok = checkRange(intrMat) && checkRange(distCoeffs);

	if (ok)
	{
		cout << "done with RMS error=" << rms << endl;
		return true;
	}
	else
		return false;
}

static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners)
{
	corners.resize(0);
	for (int i = 0; i < boardSize.height; i++)        //height和width位置不能颠倒
	for (int j = 0; j < boardSize.width; j++)
	{
		corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
	}
}