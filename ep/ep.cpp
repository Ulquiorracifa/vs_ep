// ep.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <highgui.h>
#include <cv.h>
#include <cxcore.h>
#include <iostream>
using namespace std;
using namespace cv;
#include <opencv2/opencv.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\calib3d\calib3d.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\legacy\legacy.hpp>
#include "opencv2/contrib/contrib.hpp" 

#include"SAD.h"
#include "rec.h"

#pragma comment(lib,"opencv_highgui2413d.lib")    
#pragma comment(lib,"opencv_core2413d.lib")    
#pragma comment(lib,"opencv_imgproc2413d.lib")  

int getDisparityImage(cv::Mat& disparity, cv::Mat& disparityImage, bool isColor);
int StereoGC(IplImage* img1, IplImage* img2);
void BM();

int _tmain(int argc, _TCHAR* argv[])
{
	rec();
	//IplImage * img1 = cvLoadImage("D:\\Development\\pycharm_workspace\\EpipolarRec\\expData\\lresize.jpg", 0);
	//IplImage * img2 = cvLoadImage("D:\\Development\\pycharm_workspace\\EpipolarRec\\expData\\rresize.jpg", 0);
	//CvStereoGCState* GCState = cvCreateStereoGCState(64, 3);
	//assert(GCState);
	//cout << "start matching using GC" << endl;
	//CvMat* gcdispleft = cvCreateMat(img1->height, img1->width, CV_16S);
	//CvMat* gcdispright = cvCreateMat(img2->height, img2->width, CV_16S);
	//CvMat* gcvdisp = cvCreateMat(img1->height, img1->width, CV_8U);
	//int64 t = getTickCount();
	//cvFindStereoCorrespondenceGC(img1, img2, gcdispleft, gcdispright, GCState);
	//t = getTickCount() - t;
	//cout << "Time elapsed:" << t * 1000 / getTickFrequency() << endl;
	////cvNormalize(gcdispleft,gcvdisp,0,255,CV_MINMAX);
	////cvSaveImage("GC_left_disparity.png",gcvdisp);
	//cvNormalize(gcdispright, gcvdisp, 0, 255, CV_MINMAX);
	//cvSaveImage("GC_right_disparity.png", gcvdisp);


	//cvNamedWindow("GC_disparity", 0);
	//cvShowImage("GC_disparity", gcvdisp);
	//cvWaitKey(0);
	//cvReleaseMat(&gcdispleft);
	//cvReleaseMat(&gcdispright);
	//cvReleaseMat(&gcvdisp);


	//SGBM
	/*IplImage * img1 = cvLoadImage("D:\\Development\\pycharm_workspace\\EpipolarRec\\expData\\lresize.jpg", 0);
	IplImage * img2 = cvLoadImage("D:\\Development\\pycharm_workspace\\EpipolarRec\\expData\\rresize.jpg", 0);
	cv::StereoSGBM sgbm;
	int SADWindowSize = 9;
	sgbm.preFilterCap = 63;
	sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;
	int cn = img1->nChannels;
	int numberOfDisparities = 64;
	sgbm.P1 = 8 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.P2 = 32 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.minDisparity = 0;
	sgbm.numberOfDisparities = numberOfDisparities;
	sgbm.uniquenessRatio = 10;
	sgbm.speckleWindowSize = 100;
	sgbm.speckleRange = 32;
	sgbm.disp12MaxDiff = 1;
	Mat disp, disp8;
	int64 t = getTickCount();
	sgbm((Mat)img1, (Mat)img2, disp);
	t = getTickCount() - t;
	cout << "Time elapsed:" << t * 1000 / getTickFrequency() << endl;
	disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));

	namedWindow("left", CV_WINDOW_NORMAL);
	cvShowImage("left", img1);
	namedWindow("right", CV_WINDOW_NORMAL);
	cvShowImage("right", img2);
	namedWindow("disparity", CV_WINDOW_NORMAL);
	imshow("disparity", disp8);
	waitKey();
	imwrite("sgbm_disparity.png", disp8);
	cvDestroyAllWindows();
	return 0;*/


	/*IplImage * img1 = cvLoadImage("D:\\Development\\pycharm_workspace\\EpipolarRec\\expData\\l.jpg", 0);
	IplImage * img2 = cvLoadImage("D:\\Development\\pycharm_workspace\\EpipolarRec\\expData\\r.jpg", 0);

	StereoGC(img1, img2);*/

	//BM();

	//Mat Img_L = imread("D:\\Development\\pycharm_workspace\\EpipolarRec\\expData\\l.jpg", 0);
	//Mat Img_R = imread("D:\\Development\\pycharm_workspace\\EpipolarRec\\expData\\r.jpg", 0);
	//Mat Disparity;    //视差图

	////SAD mySAD;
	//SAD mySAD(7, 30);
	//Disparity = mySAD.computerSAD(Img_L, Img_R);


	//namedWindow("Img_L", CV_WINDOW_NORMAL);
	//namedWindow("Img_R", CV_WINDOW_NORMAL);
	//namedWindow("Disparity", CV_WINDOW_NORMAL);

	//imshow("Img_L", Img_L);
	//imshow("Img_R", Img_R);
	//imshow("Disparity", Disparity);
	//waitKey();
	//return 0;


}


int StereoGC(IplImage* img1, IplImage* img2)
{
	IplImage* imgl = cvCreateImage(cvGetSize(img1), img1->depth, 1);//不可以仅仅设置为IplImage* imgl = NULL	
	IplImage* imgr = cvCreateImage(cvGetSize(img2), img2->depth, 1);
	cvCvtColor(img1, imgl, CV_BGR2GRAY);
	cvCvtColor(img2, imgr, CV_BGR2GRAY);

	CvStereoGCState* state = cvCreateStereoGCState(64, 3);
	assert(state);
	CvMat* left_disp_ = cvCreateMat(imgl->height, imgl->width, CV_16S);
	CvMat* right_disp_ = cvCreateMat(imgr->height, imgr->width, CV_16S);
	CvMat* gcvdisp = cvCreateMat(imgl->height, imgl->width, CV_8U);
	cvFindStereoCorrespondenceGC(imgl, imgr, left_disp_, right_disp_, state, 0);

	cvNormalize(right_disp_, gcvdisp, 0, 255, CV_MINMAX);

	cv::Mat di;
	CvMat *a = gcvdisp;
	Mat b = Mat(a, true);
	getDisparityImage(b, di, true);	//转换为彩色视图差

	cvWaitKey(0);
	cvReleaseMat(&left_disp_);
	cvReleaseMat(&right_disp_);
	cvReleaseMat(&gcvdisp);
	cvReleaseStereoGCState(&state);
	return 1;
}

int getDisparityImage(cv::Mat& disparity, cv::Mat& disparityImage, bool isColor)
{
	// 将原始视差数据的位深转换为 8 位
	cv::Mat disp8u;
	if (disparity.depth() != CV_8U)
	{
		disparity.convertTo(disp8u, CV_8U, 255 / (64 * 16.));
	}
	else
	{
		disp8u = disparity;
	}


	// 转换为伪彩色图像 或 灰度图像
	if (isColor)
	{
		if (disparityImage.empty() || disparityImage.type() != CV_8UC3)
		{
			disparityImage = cv::Mat::zeros(disparity.rows, disparity.cols, CV_8UC3);
		}


		for (int y = 0; y<disparity.rows; y++)
		{
			for (int x = 0; x<disparity.cols; x++)
			{
				uchar val = disp8u.at<uchar>(y, x);
				uchar r, g, b;


				if (val == 0)
					r = g = b = 0;
				else
				{
					r = 255 - val;
					g = val < 128 ? val * 2 : (uchar)((255 - val) * 2);
					b = val;
				}
				disparityImage.at<cv::Vec3b>(y, x) = cv::Vec3b(r, g, b);
			}
		}
	}
	else
	{
		disp8u.copyTo(disparityImage);
	}




	IplImage * Image;
	Image = &IplImage(disparityImage);
	cvSaveImage("colordisparityImage.png", Image);
	namedWindow("3、disparityImage", 1);
	cvShowImage("3、disparityImage", Image);
	waitKey(0);
	return 1;
}

void BM()
{
	IplImage * img1 = cvLoadImage("D:\\Development\\pycharm_workspace\\EpipolarRec\\expData\\lresize.jpg", 0);
	IplImage * img2 = cvLoadImage("D:\\Development\\pycharm_workspace\\EpipolarRec\\expData\\rresize.jpg", 0);
	CvStereoBMState* BMState = cvCreateStereoBMState();
	assert(BMState);
	BMState->preFilterSize = 9;
	BMState->preFilterCap = 31;
	BMState->SADWindowSize = 15;
	BMState->minDisparity = 0;
	BMState->numberOfDisparities = 64;
	BMState->textureThreshold = 10;
	BMState->uniquenessRatio = 15;
	BMState->speckleWindowSize = 100;
	BMState->speckleRange = 32;
	BMState->disp12MaxDiff = 1;

	CvMat* disp = cvCreateMat(img1->height, img1->width, CV_16S);
	CvMat* vdisp = cvCreateMat(img1->height, img1->width, CV_8U);
	int64 t = getTickCount();
	cvFindStereoCorrespondenceBM(img1, img2, disp, BMState);
	t = getTickCount() - t;
	cout << "Time elapsed:" << t * 1000 / getTickFrequency() << endl;
	cvSave("disp.xml", disp);
	cvNormalize(disp, vdisp, 0, 255, CV_MINMAX);
	cvNamedWindow("BM_disparity", 0);
	cvShowImage("BM_disparity", vdisp);
	cvWaitKey(0);
	//cvSaveImage("cones\\BM_disparity.png",vdisp);
	cvReleaseMat(&disp);
	cvReleaseMat(&vdisp);
	cvDestroyWindow("BM_disparity");
}


