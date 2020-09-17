//////////////////////////////////////////////////////////////////////////////////////////OPENCV CODE FOR POSITION, AND VELOCITY ESTIMATION/////////////////////////////////////////////////////////////////////////////////////////

//***********************************************************************LIBRARIES NEEDED***************************************************************************************//
//*****************************************************************************************************************************************************************************//




//////////////Include for OpenCv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

//////////////Include for General operations 
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "Serial.h"
#include <winbase.h>
#include <windows.h>
#include <WinUser.h>
#include <Windows.h>
#include <conio.h>
#include <ctype.h>
#include <chrono>
#include "stdafx.h"
#include <cstdlib>
#include <ctime>
#include "resource.h"
#include <sstream>
#include <complex>
#include <iomanip>
#include <string.h>
#include <fstream>  
#include <math.h>
#include "matrix.h"

//////////////Include for Threading

#include <thread>
#include <mutex> 
#include <condition_variable>
#include "common.h" //shared global variables



//***********************************************************************DECLARATIONS***************************************************************************************//
//*************************************************************************************************************************************************************************//

#define MAX_LOADSTRING			100
#define IDS_APP_TITLE           103
#define IDC_GET_POSITION        109
#define IDI_GET_POSITION        107

#define drawCross( center, color, d )                                 \
line(frameROI, Point(center.x - d, center.y - d), Point(center.x + d, center.y + d), color, 2, CV_AA, 0); \
line(frameROI, Point(center.x + d, center.y - d), Point(center.x - d, center.y + d), color, 2, CV_AA, 0)


using namespace cv;
using namespace std;



// Serial Port 
CSerial tCom232;

const double pi = 3.14159265358979323846;

// Window names Definitions 

string  wnd2 = "Markers", wnd3 = "Position/Velocity";
clock_t start;    // clock to calculate the processing time	

// Interface-Making 

Mat display = Mat::zeros(298, 640, CV_8UC3);
Mat text = Mat::zeros(298, 640, CV_8UC3);        //because must match frameROI, which comes from rowRange(50, 388) 388-50=338

char * etext = "pala.ttf";// BRITANIC.ttf";
HINSTANCE hInst;								// current instance
TCHAR szTitle[MAX_LOADSTRING];					// The title bar text
TCHAR szWindowClass[MAX_LOADSTRING];			// the main window class name
HWND hWnd, button, h_startButton;
TCHAR lpszPassword[16];


// Forward declarations of functions included in this code module:
ATOM				MyRegisterClass(HINSTANCE hInstance);
BOOL				InitInstance(HINSTANCE, int);
LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);


int APIENTRY _tWinMain(_In_ HINSTANCE hInstance,
	_In_opt_ HINSTANCE hPrevInstance,
	_In_ LPTSTR    lpCmdLine,
	_In_ int       nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);
	MSG msg;
	HACCEL hAccelTable;

	// Initialize global strings
	MyRegisterClass(hInstance);

	// Perform application initialization:
	if (!InitInstance(hInstance, nCmdShow))
	{
		return FALSE;
	}

	hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_GET_POSITION));

	// Main message loop:

	BOOL bRet;

	while ((bRet = GetMessage(&msg, (HWND)NULL, 0, 0)) != 0)
	{
		if (bRet != -1)
		{
			if (TranslateAccelerator(hWnd, hAccelTable, &msg) == 0)
			{
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
		}
	}

	return (int)msg.wParam;
}

vector<Point> PXe; // List of Points for plotting path
vector<Point> PXeF; // List of Points for plotting path

//***********************************************************************FUNCTION DECLARATIONS*****************************************************************************//
//*************************************************************************************************************************************************************************//

ATOM MyRegisterClass(HINSTANCE hInstance)
{
	WNDCLASSEX wcex;

	wcex.cbSize = sizeof(WNDCLASSEX);

	wcex.style = CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc = WndProc;
	wcex.cbClsExtra = 0;
	wcex.cbWndExtra = 0;
	wcex.hInstance = hInstance;
	wcex.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_GET_POSITION));
	wcex.hCursor = LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
	wcex.lpszMenuName = NULL;//MAKEINTRESOURCE(IDC_WIN32PRJ);
	wcex.lpszClassName = szWindowClass;
	//wcex.hIconSm = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

	return RegisterClassEx(&wcex);
}

//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	int wmId, wmEvent;
	HDC hdc;

	switch (message)
	{
	case WM_CREATE:
	{
		button = CreateWindow(L"BUTTON", L"EXIT", BS_OWNERDRAW, 100, 200, 50, 50, NULL, (HMENU)IDC_MAIN_BUTTON, hInst, NULL);
		h_startButton = CreateWindow(L"BUTTON", L"START", BS_OWNERDRAW, 200, 100, 50, 50, NULL, (HMENU)IDC_BUTTON, hInst, NULL);
		break;
	}

	case WM_COMMAND:
		wmId = LOWORD(wParam);
		wmEvent = HIWORD(wParam);
		// Parse the menu selections:
		switch (wmId)
		{

		case IDC_MAIN_BUTTON:
			//DialogBox(hInst, MAKEINTRESOURCE(IDD_DIALOG1), hWnd, PasswordProc);

			break;

		case IDC_BUTTON:
			//DialogBox(hInst, MAKEINTRESOURCE(IDD_DIALOG2), hWnd, startButtonProc);
			;
		}
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;

}

//DRAW WRAPED TEXT
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
	hInst = hInstance; // Store instance handle in our global variable

	hWnd = CreateWindow(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
		CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, NULL, NULL, hInstance, NULL);

	if (!hWnd)
	{
		return FALSE;
	}

	ShowWindow(hWnd, nCmdShow);
	UpdateWindow(hWnd);

	return TRUE;
}

// Gets the positions of the counturs
vector<Point2f> get_positions(Mat& image) //get_positions: a function to retrieve the center of the detected marker.
{
	if (image.channels() > 1)
	{
		//cout << "get_positions: !!! Input image must have a single channel" << endl;
		return vector<Point2f>();
	}

	vector<vector<Point> > contours;
	findContours(image, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
	for (size_t i = 0; i < contours.size(); ++i)
	{
		// Calculate the area of each contour
		double area = contourArea(contours[i]);
		// Ignore contours that are too small or too large
		if (area < 1e2 && 1e3 < area) continue;

		// Approximate contours to polygons and then get the center of the objects
		vector<vector<Point> > contours_poly(contours.size());
		vector<Point2f> center(contours.size());
		vector<float> radius(contours.size());

		for (unsigned int i = 0; i < contours.size(); i++)
		{
			approxPolyDP(Mat(contours[i]), contours_poly[i], 5, true);
			minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]);
		}
		return center;
	}
}

//VOID  FUCNTIONS
void destroy()
{
	DestroyWindow(hWnd);
	destroyWindow(wnd3);

}


float Xpdot = 0;
float Ypdot = 0;

float Xpddot = 0;
float Ypddot = 0;

float thetapdot = 0;

// Used for Point calculation (Fish markers)
float x11, y11, x22, y22, xe1, ye1, xeI, yeI, xe2, ye2, xc, yc;
Point centerf;              //the center point of the fish.

vector<Point> fishv, fishv2, kalmanv, kalmanv2; //Lists for Kalman Filter
vector<Point2f> xp;


////////////To Draw Desired Trajectory 
float pxx = 0;
float pxy = 0;
float pxx0 = 0;
float pxy0 = 0;
float xd1 = 0;
float xd2 = 0;
float xd3 = 0;
float xd4 = 0;
float xd5 = 0;
double tn = 0;
double currentTime = 0;
double currentTTime = 0;
float Theta2D = 0.0;
int sw = 0;

int AW = 3; // Window of averaging states
float p = 0;
int u = 0;
float sump[4] = { 0.0, 0.0, 0.0, 0.0 };

double DT = 0;

//Fish angle and error angle
float betaf = 0;
float PHI = 0;
float thetae = 0;

/// Variables for time keeping

time_t clock_start = 0;
time_t clock_startt = 0;
time_t clock_end = 0;
time_t clock_endt = 0;
time_t clock_starts = 0;
time_t clock_ends = 0;

/// Declaration of fish command
char cmd[11] = { '1', '0', '0', '.', '0', '0', '0', '0', '.', '0', '0' };

#ifndef M_PI
const double M_PI = (double) 3.14159265358979323846;		// per CRC handbook, 14th. ed.
#endif

#ifndef M_PI_2
const double M_PI_2 = (M_PI / 2.0);				// PI/2
#endif

#ifndef M_2PI
const double M_2PI = (M_PI*2.0);				// PI*2
#endif
/////////////////////////////////////////////////////////////////Function to calculate desired trajectory

float DesiredTrajectory(float *R1, float *R2, float *k1, float *k2, double *t, float *td, char *s, float *xd1, float *xd2, float *xd3, float *xd4, float *xd5)

{
	if (*s == 'c')
	{

		Xpdot = *R1* *k1*cos(*k1* *t);
		Ypdot = -*R1* *k1*sin(*k1* *t);

		Xpddot = -*R1* *k1* *k1*sin(*k1* *t);
		Ypddot = -*R1* *k1* *k1*cos(*k1* *t);

		thetapdot = (Ypddot*Xpdot - Xpddot*Ypdot) / (Ypdot*Ypdot + Xpdot*Xpdot);

		*xd1 = cos(*td)*Xpdot + sin(*td)*Ypdot;
		*xd2 = -sin(*td)*Xpdot + cos(*td)*Ypdot;
		*xd3 = thetapdot;
		*xd4 = cos(*td)*Xpddot + sin(*td)*Ypddot + *xd3* (-sin(*td)*Xpdot + cos(*td)*Ypdot);
		*xd5 = -sin(*td)*Xpddot + cos(*td)*Ypddot + *xd3 * (-cos(*td)*Xpdot - sin(*td)*Ypdot);

	}

	else if (*s == '8')
	{

		Xpdot = *R1* *k1*cos(*k1* *t);
		Ypdot = *R2* *k2*cos(*k2* *t);

		Xpddot = -*R1**k1* *k1*sin(*k1* *t);
		Ypddot = -*R2* *k2* *k2*sin(*k2* *t);

		thetapdot = (Ypddot*Xpdot - Xpddot*Ypdot) / (Ypdot*Ypdot + Xpdot*Xpdot);

		*xd1 = cos(*td)*Xpdot + sin(*td)*Ypdot;
		*xd2 = -sin(*td)*Xpdot + cos(*td)*Ypdot;
		*xd3 = thetapdot;
		*xd4 = cos(*td)*Xpddot + sin(*td)*Ypddot + *xd3 * (-sin(*td)*Xpdot + cos(*td)*Ypdot);
		*xd5 = -sin(*td)*Xpddot + cos(*td)*Ypddot + *xd3 * (-cos(*td)*Xpdot - sin(*td)*Ypdot);

	}
	else if (*s == 'v')
	{
		Xpdot = 0.03;
		*R2 = 0.3;
		*k2 = 0.3;
		Ypdot = *R2* *k2*cos(*k2* *t);

		Xpddot = 0;
		Ypddot = -*R2* *k2* *k2*sin(*k2* *t);

		thetapdot = (Ypddot*Xpdot - Xpddot*Ypdot) / (Ypdot*Ypdot + Xpdot*Xpdot);

		*xd1 = cos(*td)*Xpdot + sin(*td)*Ypdot;
		*xd2 = -sin(*td)*Xpdot + cos(*td)*Ypdot;
		*xd3 = thetapdot;
		*xd4 = cos(*td)*Xpddot + sin(*td)*Ypddot + *xd3 * (-sin(*td)*Xpdot + cos(*td)*Ypdot);
		*xd5 = -sin(*td)*Xpddot + cos(*td)*Ypddot + *xd3 * (-cos(*td)*Xpdot - sin(*td)*Ypdot);
	}
	return 0;
}


inline double constrainAngle(double x){
	x = fmod(x + M_PI, M_2PI);
	if (x < 0)
		x += M_2PI;
	return x - M_PI;
}
// convert to [-360,360]
inline double angleConv(double angle){
	return fmod(constrainAngle(angle), M_2PI);
}
inline double angleDiff(double a, double b){
	double dif = fmod(b - a + M_PI, M_2PI);
	if (dif < 0)
		dif += M_2PI;
	return dif - M_PI;
}
inline double unwrap(double previousAngle, double newAngle){
	return previousAngle - angleDiff(newAngle, angleConv(previousAngle));
}



//****************************************************************************MAINPROGRAMFUCTION**********************************************************************************//
//*******************************************************************************************************************************************************************************//


int position(char cmd[], float *xe, float *ye, float *Beta, float *Uf, float *Vf, float *Omegaf, float *UD, float *VD, float *OmegaD, float *Uddot, float *Vddot, int *Stop, float *ThetaD, float *xp, float *yp, float *xf, float *yf, float *Betaf, double *alpha_a, double *alpha_0, double *tim)

{
	// Local Declarations

	double alpha_alast = 0.0;
	double alpha_0last = 0.0;
	float ufi = 0;
	float vfi = 0.0;
	float Omegai = 0.0;
	float ufi2 = 0;
	float vfi2 = 0.0;

	double dummyt = 0; 
	double dummyt_const = 0;
	bool tt = true;
	int first = 1;

	int m = 1;
	int n = 0;
	int i = 0;
	int yy = 0;
	int q = 0;

	int nm = 1;
	float PHI1 = 0;
	float Omega1 = 0;
	int nn = 1;
	bool av;
	bool aT = false;
	float dts = 0;
	Point2f statePt;

	std::vector<double> x0(18);
	std::vector<double> x0a(5);
	float mats[1000][17];
	mats[0][10] = 0.0;
	float matsa[1000][10];
	
	double control_tolerance = 1 * pi / 180.0;
	bool CCon = true;


	Mat frame, frame1, hsv_frame, Blue_Marker, Yellow_Marker; //Declare Arrays 

	///////////////////////////////////  OPEN SERIAL PORT:  /////////////////////////////////////  

	if (!tCom232.Open(3, 9600))
	{
		printf("Serial Port open failed\n");
		return -1;
	}
	else
	{
		printf("Serial Port open succeeded\n");
		tCom232.ClearSysBuffer();
	}
	
	int ContCha = 0; 
	///////////////////////////////////  OPEN THE CAPTURE DEVICE:  ///////////////////////////////////

	VideoCapture cap(0); //0 for camera 1 for webcam
	namedWindow(wnd3, CV_WINDOW_NORMAL); //Creates a window named wnd3 with option to resize window size.
	cvSetWindowProperty("Position/Velocity", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN); //Changes parameters of a window dynamically.
	
	// Changing the resolution of the camera
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 870);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 540);

	int frame_width = static_cast<int>((cap.get(CV_CAP_PROP_FRAME_WIDTH)));
	int frame_height = static_cast<int>((cap.get(CV_CAP_PROP_FRAME_HEIGHT)));

	if (!cap.isOpened())
	{
		cout << "!!! Failed to open webcam" << endl;
		return -1; //break;
	}

	///////////////////////////////////  KALMAN DEFINITION: ///////////////////////////////////

	KalmanFilter KF(6, 3, 0); // Define Kalman Filter with 6 states and 3 measurements

	double T = 3;
	double dt = 1 / T; /// dt for Kalman Filter

	KF.transitionMatrix = (Mat_<float>(6, 6));// << 1, 0, 0, dt, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1); //Transition Matrix
	setIdentity(KF.transitionMatrix);
	KF.transitionMatrix.at<float>(0, 2) = dt;
	KF.transitionMatrix.at<float>(1, 3) = dt;
	KF.transitionMatrix.at<float>(4, 5) = dt;

	KF.measurementMatrix = (Mat_<float>(3, 6));
	setIdentity(KF.measurementMatrix);

	KF.measurementMatrix.at<float>(0, 0) = 1;  // x
	KF.measurementMatrix.at<float>(1, 1) = 1;  // y
	KF.measurementMatrix.at<float>(2, 2) = 0;
	KF.measurementMatrix.at<float>(2, 4) = 1;  // phi

	Mat_<float> measurement(3, 1); measurement.setTo(Scalar(0)); // Create array 3x1 with measurements.


	/////////////////////////////////// Intrinsic and Extrinsic Camera Values ///////////////////////////////////


	Mat intrinsic = *(Mat_<double>(3, 3, CV_32FC1) << 5.8416542080814600e+002, 0, 4.3491729516948988e+002, 0, 5.8305874940664262e+002, 2.4345242369328125e+002, 0, 0, 1);

	Mat rotationMatrix = *(Mat_<double>(3, 3) << -1.2862809765696426e-002, -9.9952653077205689e-001, -2.7951071673686476e-002, -9.9824173942811134e-001, 1.4453802443491703e-002, -5.7484930707647229e-002, 5.7861712629942066e-002, 2.7162508678333898e-002, -9.9795501919366558e-001);

	Mat tvec = *(Mat_<double>(3, 1) << 7.6833469963638277e+002, 4.0798109981338189e+002, 1.1512212953589342e+003);

	Mat rvec = *(Mat_<double>(1, 3) << 2.1637284100393699e+000, -2.1935165515838655e+000, 3.2841389556885439e-002);

	Mat distCoeffs = *(Mat_<double>(1, 5) << 8.0007794752334632e-002, -2.4292991634217262e-001, 6.7870226181363826e-004, -2.2813304886926040e-003, 1.9816461254179935e-001);

	double sS = 9.0601371070937728e+002;


	/////////////////////////////////// DECLARION AND PROJECTION OF INERTIAL AXIS ///////////////////////////////////

	std::vector<cv::Point2d> imagePoints;
	std::vector<cv::Point3d> objectPoints;
	objectPoints.push_back(cv::Point3d(0, 0, 0));
	objectPoints.push_back(cv::Point3d(90, 0, 0));
	objectPoints.push_back(cv::Point3d(0, 90, 0));

	projectPoints(objectPoints, rvec, tvec, intrinsic, distCoeffs, imagePoints);

	Point or(imagePoints[0].x, imagePoints[0].y);
	Point px(imagePoints[1].x, imagePoints[1].y);
	Point py(imagePoints[2].x, imagePoints[2].y);

	/////////////////////////////////// storing exerimental Results without control portion ///////////////////////////////////

	cv::Mat uvPointze = cv::Mat::ones(3, 1, cv::DataType<double>::type); // Store Data of interest
	cv::Mat uvPoint11 = cv::Mat::ones(3, 1, cv::DataType<double>::type); // Mat to store front point of fish to calculate PHI 
	cv::Mat UVPoint11 = cv::Mat::ones(3, 1, cv::DataType<double>::type); // creates 3x1 vector: [u,v,1]^T
	cv::Mat uvPoint = cv::Mat::ones(3, 1, cv::DataType<double>::type); // creates 3x1 vector: [u,v,1]^T
	cv::Mat UVPoint = cv::Mat::ones(3, 1, cv::DataType<double>::type); // creates 3x1 vector: [u,v,1]^T
	*tim = 0.0;
	double samp = 0.0;

	// Variable for storing video frames
	Mat frameg;

	//std::ofstream outputO("PlottingNMPCExperiment/matrixO.txt");
	std::ostringstream filename;
	filename << "Results/matrixO" << ss << Ra * 100 << ud * 100 << ".txt";
	string ssb = filename.str();
	ofstream outputO(ssb);

	//std::ofstream outputOa("PlottingNMPCExperiment/matrixOa.txt");
	std::ostringstream filenameA;
	filenameA << "Results/matrixOa" << ss << Ra * 100 << ud * 100 << ".txt";
	string ssa = filenameA.str();
	ofstream outputOa(ssa);
    
	std::ostringstream filenameV;
	filenameV << "Results/video" << ss << Ra * 100 << ud * 100 << ".avi";
	string ssv = filenameV.str();

	
	/////////////////////////////////// MAIN LOOP ///////////////////////////////////

	
	VideoWriter oVideoWriter(ssv, CV_FOURCC('F','M', 'P', '4'), 3, Size(frame_width, frame_height)); //This codec works as well
	//VideoWriter oVideoWriter(ssv, CV_FOURCC('D','I','V','3'), 3, Size(frame_width, frame_height));

	while (!*(int*)Stop)
	{
		
		if (u == 0){
			clock_start = clock();
		}

		/////////////////////////////////// IMAGE PROCESSING ///////////////////////////////////
		cap >> frame;
		if (!cap.read(frame)) // Retrieve a new frame from the camera
			break;


		undistort(frame, frame1, intrinsic, distCoeffs); //undistort image with calculated camera coefficients
		Mat frameROI = frame1.rowRange(10, 480); // Takes a part of the Mat frame. 

		//cout << frameROI.size();

		/*Create black areas in program to block areas not to be displayed*/

		Rect roi = Rect(0, 0, 864, 90); //Creates Top black box
		Rect roi3 = Rect(0, 430, 864, 40);//Created Bottome Black bos
		Rect roil = Rect(0, 0, 82, 450); //Creates left black box
		Rect roi4 = Rect(782, 0, 82, 450); //Creates right black box

		Mat buttonROI = frameROI(roi);
		Mat grayROI3 = frameROI(roi3);
		Mat grayROIl = frameROI(roil);
		Mat grayROI4 = frameROI(roi4);

		buttonROI.setTo(Scalar(0, 0, 0));//CV_RGB(110, 110, 110); this where color should go?
		grayROI3.setTo(Scalar(0, 0, 0));
		grayROIl.setTo(Scalar(0, 0, 0));
		grayROI4.setTo(Scalar(0, 0, 0));

		// Convert BGR frame to HSV to be easier to separate the colors

		cvtColor(frameROI, hsv_frame, CV_BGR2HSV);


		// Isolate blue colored marker and save them in a binary image

		inRange(hsv_frame, Scalar(101, 55, 103), Scalar(133, 245, 255), Blue_Marker); //Threshold the image

		//morphological opening (remove small objects from the foreground)

		erode(Blue_Marker, Blue_Marker, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(Blue_Marker, Blue_Marker, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (fill small holes in the foreground)

		dilate(Blue_Marker, Blue_Marker, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(Blue_Marker, Blue_Marker, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		// bluring

		GaussianBlur(Blue_Marker, Blue_Marker, Size(9, 9), 1.5);

		// Isolate red colored marker and save them in a binary image

		inRange(hsv_frame, Scalar(0, 100, 100), Scalar(10, 255, 255), Yellow_Marker); //Threshold the image

		//morphological opening (remove small objects from the foreground)

		erode(Yellow_Marker, Yellow_Marker, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(Yellow_Marker, Yellow_Marker, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (fill small holes in the foreground)

		dilate(Yellow_Marker, Yellow_Marker, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(Yellow_Marker, Yellow_Marker, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		// bluring

		GaussianBlur(Yellow_Marker, Yellow_Marker, Size(9, 9), 1.5);

		vector<Point2f> points1 = get_positions(Yellow_Marker);

		// Draw a small green circle at the location of Red_Markers

		for (unsigned int i = 0; i < points1.size(); i++){
			circle(frameROI, points1[0], 3, Scalar(0, 255, 0), -1, 8, 0);
			x11 = points1[i].x;
			y11 = points1[i].y;
		}

		// Retrieve a vector of points with the (x,y) locations of the Blue_Marker2

		vector<Point2f> points2 = get_positions(Blue_Marker);

		// Draw a small green circle at the locations of the Blue_Markers

		for (unsigned int i = 0; i < points2.size(); i++){
			circle(frameROI, points2[0], 3, cv::Scalar(0, 255, 0), -1, 8, 0);
			x22 = points2[i].x;
			y22 = points2[i].y;
		}


		// Get the center point of the fish.

		xc = (x22 + x11) / 2;
		yc = (y22 + y11) / 2;
		centerf.x = xc;       // the center of the fish x coordinate
		centerf.y = yc;		 // // the center of the fish y coordinate


		// Draw a small green circle at the location of the center of the fish.

		circle(frameROI, centerf, 3, cv::Scalar(0, 255, 0), -1, 8, 0);    // the center of the fish

		/////////////////////////////////// DRAW THE COORDINATE AXIS: ///////////////////////////////////

		/*	arrowedLine(frame1, or, px, Scalar(255, 0, 0), 5, 8, 0, 0.3);
		arrowedLine(frame1, or, py, Scalar(255, 0, 0), 5, 8, 0, 0.3);
		putText(frameROI, "Y", Point(px.x - 20, px.y - 2), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 200, 200), 4);// x label
		putText(frameROI, "X", Point(py.x + 2, py.y - 25), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 200, 200), 4);// y label*/



		/////////////////////////////////// CONVERT FISH POSITION TO WORLD COORDINATES: ///////////////////////////////////

		uvPoint.at<double>(0, 0) = xc;
		uvPoint.at<double>(1, 0) = yc;

		cv::Mat tempMat, tempMat2;

		tempMat = rotationMatrix.inv() *  intrinsic.inv() * uvPoint;
		tempMat2 = rotationMatrix.inv() * tvec;
		sS = 0 + tempMat2.at<double>(2, 0); //-127 represents the height Zconst
		sS /= tempMat.at<double>(2, 0);
		UVPoint = rotationMatrix.inv() * (sS *  intrinsic.inv() * uvPoint - tvec); //Converts  point to real world coordinatees

		clock_endt = clock();
		float tt = (double)(clock_endt - clock_startt) / CLOCKS_PER_SEC;
		clock_startt = clock();


		/////////////////////////////////// CALCULATION OF FISH HEADING ANGLE: ///////////////////////////////////

		uvPoint11.at<double>(0, 0) = x11;
		uvPoint11.at<double>(1, 0) = y11;

		tempMat = rotationMatrix.inv() *  intrinsic.inv() * uvPoint11;
		tempMat2 = rotationMatrix.inv() * tvec;
		sS = 0 + tempMat2.at<double>(2, 0); //-127 represents the height Zconst
		sS /= tempMat.at<double>(2, 0);
		UVPoint11 = rotationMatrix.inv() * (sS *  intrinsic.inv() * uvPoint11 - tvec);

		// Calculation of the angle of the fish with a horizontal line through the center of it.

		xeI = UVPoint11.at<double>(1, 0) - UVPoint.at<double>(1, 0);
		yeI = UVPoint11.at<double>(0, 0) - UVPoint.at<double>(0, 0);

		PHI = atan2f(yeI, xeI);
		PHI1 = PHI;

		/////////////////////////////////// KALMN FILTER: /////////////////////////////////// 

		// Initiation

		if (first == 1)
		{


			//Initialize the filter. Dynamic portion inside while loop
			KF.statePre.at<float>(0) = UVPoint.at<double>(1, 0) / 1000; //xc;//  // measured  xc in world coordinates (mm)
			KF.statePre.at<float>(1) = UVPoint.at<double>(0, 0) / 1000; //yc;// //// measured yc in world coordinates (mm)
			KF.statePre.at<float>(2) = 0; // x Velocity
			KF.statePre.at<float>(3) = 0; // y velocity
			KF.statePre.at<float>(4) = PHI;
			KF.statePre.at<float>(5) = 0; // Angular Velocity in degrees/second


			//setIdentity(KF.measurementMatrix); // Create Measurement Matrix
			setIdentity(KF.processNoiseCov, Scalar::all(1e-20)); // Covariance Matrix -14

			//KF.processNoiseCov.at<float>(4, 4) = 1.0e-25;
			//KF.processNoiseCov.at<float>(5, 5) = 1.0e-15;



			setIdentity(KF.measurementNoiseCov, Scalar::all(1e-30)); // Measurement Covariance Matrix --> helps with faster convergence and coorecct position prediction-23

			//KF.measurementNoiseCov.at<float>(2 ,2) = 1e-40;

			setIdentity(KF.errorCovPost, Scalar::all(1e-30));// --> initial transient estimate?-23
			KF.errorCovPost.at<float>(2, 2) = 1e-35;

			fishv.clear();
			fishv2.clear();
			kalmanv.clear();
			first++;
		}

		// First predict, to update the internal statePre variable

		Mat prediction = KF.predict();
		//Point2f predictPt(prediction.at<float>(0), prediction.at<float>(1)); //Position
		//Point2f predictPt2(prediction.at<float>(4), 0); //PHI 


		// Get fish point

		measurement(0) = UVPoint.at<double>(1, 0) / 1000; // xc;//  // measured in word coordinates(m)
		measurement(1) = UVPoint.at<double>(0, 0) / 1000; // yc;//  // measured in word coordinates(m)


		/// Angle Unwrapping			
		if (i >= 1)
		{
			/// CounterClockwise Rotation

			/*if (0 <= PHI && PHI <= pi)
				PHI1 = PHI;

			if (PHI1 <0 && mats[u - 1][9] <= pi)
			{

				PHI1 = mats[u - 1][9] + abs(abs(mats[u - 1][6]) - abs(PHI));
			}
			else if (mats[u - 1][9] >= 1.1*pi && (0 >= PHI1 && PHI1 >= -pi))
			{
				PHI1 = mats[u - 1][9] + (abs(mats[u - 1][6]) - abs(PHI));
			}
			else if (mats[u - 1][9] >= 1.1*pi && (0 <= PHI))
			{
				PHI1 = mats[u - 1][9] + abs(abs(mats[u - 1][6]) - abs(PHI));
			}


			//Jumped from positive to negative. 2nd-3rd quadrant 
			if (PHI1 < 0 && (mats[u - 1][9] >= .75*pi))
			{
				PHI1 = mats[u - 1][9] + (-abs(PHI) + abs(mats[u - 1][6]));
			}
			else if (mats[u - 1][9] >= 1.4*pi && (0 <= PHI && PHI <= pi))
			{
				PHI1 = mats[u - 1][9] + abs(-abs(PHI) + abs(mats[u - 1][6]));
			}


			//// Clockwise Rotation

			if (0 >= PHI && PHI >= -pi)
				PHI1 = PHI;

			////// If PHI bigger than zero

			if (PHI1 >= 0 && mats[u - 1][9] <= -.75*pi)
			{

				PHI1 = mats[u - 1][9] - abs(abs(mats[u - 1][6]) - abs(PHI));
			}
			else if (mats[u - 1][9] <= -1.7*pi && (0 >= PHI1 && PHI1 >= -pi))
			{
				PHI1 = mats[u - 1][9] + (abs(mats[u - 1][6]) - abs(PHI));
			}
			else if (mats[u - 1][9] <= -1.7*pi && (0 <= PHI) && mats[u-1][3]<0.05)
			{
				PHI1 = mats[u - 1][9] - abs(abs(mats[u - 1][6]) - abs(PHI));
			}

			if (PHI1 <= 0 && mats[u - 1][9] >= .5*pi)
			{
				PHI1 = -(pi - abs(PHI));
			}*/


			PHI1= unwrap(mats[u - 1][9], PHI);

		}

		measurement(2) = PHI1;// Radians

		// The update phase

		Mat estimated = KF.correct(measurement); // Correct estimate from measurement


		Point2f statePt(estimated.at<float>(0) * 1000, estimated.at<float>(1) * 1000); //Estimate Position
		Point2f statePt2(estimated.at<float>(4), 0); //Estimated Heading Angle

		Point2f measPt(measurement(0), measurement(1));//Measured Position
		Point2f measPt2(measurement(2), 0); //Measure Heading Angle

		Point2f measVt(estimated.at<float>(2), estimated.at<float>(3)); // Estimated velocity-->meters
		Point2f measVt2(estimated.at<float>(5), 0); //Estimated angular velocity degrees/secon
		fishv.push_back(measPt); // Store measurements
		Point2f Vtw = (estimated.at<float>(2), estimated.at<float>(3));

		// Draw Crosses at estimated position to determine how 'correct' filter is. Since we changed filter to world coordinates, need to change back to draw on screen 

		std::vector<cv::Point2d> imagePointsState;
		std::vector<cv::Point3d> objectPointsState;

		objectPointsState.push_back(cv::Point3d(statePt.y, statePt.x, 0)); //Store Path in List objectPoints

		projectPoints(objectPointsState, rvec, tvec, intrinsic, distCoeffs, imagePointsState); // Find point in Image

		drawCross(imagePointsState[0], Scalar(255, 255, 255), 3); //Cross at estimated position
		
		

		/////////////////////////////////// INTERFACE DISPLAY: /////////////////////////////////// 

	

		if (i == 0)
		{
			ufi = measVt.x*cos(PHI) + measVt.y*sin(PHI);
			vfi = measVt.y*cos(PHI) - measVt.x*sin(PHI); 
		}
		if (i > 0)
		{

			ufi = ((UVPoint.at<double>(1, 0) - mats[i - 1][7]) / 1000 / tt)*cos(PHI) + ((UVPoint.at<double>(0, 0) - mats[i - 1][8]) / 1000 / tt)*sin(PHI); // measVt.x*cos(PHI) + measVt.y*sin(PHI); ////  // //(estimated.at<float>(3)*cos(PHI) + estimated.at<float>(4)*sin(PHI)) / 100; //Surge Velocity in meters
			vfi = ((UVPoint.at<double>(0, 0) - mats[i - 1][8]) / 1000 / tt)*cos(PHI) - ((UVPoint.at<double>(1, 0) - mats[i - 1][7]) / 1000 / tt)*sin(PHI); // measVt.y*cos(PHI) - measVt.x*sin(PHI); //(estimated.at<float>(4)*cos(PHI) - estimated.at<float>(3)*sin(PHI)) / 100; //Sway Velocity in meters
			ufi2 = measVt.x*cos(PHI) + measVt.y*sin(PHI); ////  // //(estimated.at<float>(3)*cos(PHI) + estimated.at<float>(4)*sin(PHI)) / 100; //Surge Velocity in meters
			vfi2 = measVt.y*cos(PHI) - measVt.x*sin(PHI); //(estimated.at<float>(4)*cos(PHI) - estimated.at<float>(3)*sin(PHI)) / 100; //Sway Velocity in meters
		
		}
		
		Omegai = estimated.at<float>(5); // Angular Velocity in radians/sec
		float alphaf = atan(vfi / ufi);



		/////////////////////////////////// TRAJECTORY DECLARATION: ///////////////////////////////////

		//-------------------------------////////////////////////////////////////// DEFINE DESIRED TRAJECTORY////////////////////////// -------------------------------//
	
		if (i <= 2)
		{
			dummyt = 0;
			dummyt_const = 0; 

		}

		if (i > 2)
		{
			clock_ends = clock();
			dts = (double)(clock_ends - clock_starts) / CLOCKS_PER_SEC;
			currentTime = currentTime + dts;
			
		}
			if (i == 2)
				currentTime = 0;
			if (ss == 'c')
			{
				//cout << "at iteration " << i << " dummyt-dummyt_const is: " << (dummyt - dummyt_const) << endl;
				*UD = 0.04; // xd1; //
				*VD = 0;
				*OmegaD = -0.15;
				*Uddot = 0;
				*Vddot = 0;
				*ThetaD = pi / 2 + *OmegaD*currentTime;//  (dummyt - dummyt_const);
				Theta2D = *ThetaD;

				if (u >= 1)
				{
					if (*OmegaD < 0 && mats[u - 1][10] <= -2.8 &&  *ThetaD >= 0)
					{
						*ThetaD = mats[u - 1][10] - abs(abs(mats[u - 1][11]) - abs(*ThetaD));
					}
					else if (mats[u - 1][10] <= -2.8 && (0 >= *ThetaD >= -pi))
					{
						*ThetaD = mats[u - 1][10] - abs(abs(mats[u - 1][11]) - abs(*ThetaD));
					}
				}

			}
			

			if (ss == 'l')

			{
				*UD = *UD;
				*VD = 0;
				*OmegaD;
				*Uddot = 0;
				*Vddot = 0;
				*ThetaD = *ThetaD + *OmegaD*dts;

				//disturbance 
				//*ThetaD = -pi;//
			}

			if (ss == 'w')
			{	
				*UD = 0.03;
				*VD = 0;
				*OmegaD = 0;
				*Uddot = 0;
				*Vddot = 0;
				*ThetaD = 45 * pi / 180 * sin(currentTime / 20 * 2 * pi);
			}

			if (ss == 'g')

			{

				if (currentTime < 16)
				{
					*UD = *UD;
					*VD = 0;
					*OmegaD;
					*Uddot = 0;
					*Vddot = 0;
					*ThetaD = *ThetaD + *OmegaD*dts;
				}

				if (currentTime >= 16)
				{
					if (sw == 1)
					{

						tn = currentTime;

					}

					currentTTime = currentTime - tn;
				
					sw = sw + 1;
					R1 = 0.25;
					R2 = R1;
					k1 = 0.2;
					k2 = k1;

					ss = 'c';

					DesiredTrajectory(&R1, &R2, &k1, &k2, &currentTTime, &*ThetaD, &ss, &xd1, &xd2, &xd3, &xd4, &xd5);

					ss = 'g';

					*UD = xd1;
					*VD = xd2;
					*OmegaD = xd3;
					*Uddot = xd4;
					*Vddot = xd5;

					*ThetaD = atan2f((sin(mats[u - 1][10])* *UD + cos(mats[u - 1][10])**VD), (cos(mats[u - 1][10])**UD - sin(mats[u - 1][10])* *VD));
					Theta2D = *ThetaD;

					if (*UD < 0)
						*UD = abs(*UD);

					if (u >= 1)
					{
						if (*OmegaD < 0 && mats[u - 1][10] <= -2.8 &&  *ThetaD >= 0)
						{
							*ThetaD = mats[u - 1][10] - abs(abs(mats[u - 1][11]) - abs(*ThetaD));
						}
						else if (mats[u - 1][10] <= -2.8 && (0 >= *ThetaD >= -pi))
						{
							*ThetaD = mats[u - 1][10] - abs(abs(mats[u - 1][11]) - abs(*ThetaD));
						}
					}
				}
			}

			if (i >= 2)
			{
				clock_starts = clock();
			}

			///////////////////////// Trajectory desired positions///////////////////////// 

			if (ss == 'l')
			{

				if (*OmegaD != 0 && i <=2 )
				{
					pxx = 300;
					pxy = 260;


				}
				else
				{

					if (i <=2)
					{
						pxx = 300;
						pxy = 360;
					}

					else
					{

						pxx = pxx + (*UD*cos(*ThetaD) + *VD*(-sin(*ThetaD)))*dts * 1000; //in mm
						pxy = pxy + (*VD*cos(*ThetaD) + *UD*(sin(*ThetaD)))*dts * 1000; // in mm

					}
				}

			}

			else if (ss == 'c')
			{
				if (i <=2)
				{
					//	pxx0 = 700;
					//pxy0 = 650;// (-800 + R1 * 1000);
					//pxx0 = 300;
					pxx0 = 300;
					pxy0 = 400;
					pxx = pxx0;
					pxy = pxy0;
				}
				else
				{

					double TurningRadius = -*UD / *OmegaD;
					//pxx = TurningRadius*cos(-*ThetaD + 3*pi / 2)*1000 + pxx0;
					//pxy = TurningRadius*1000*sin(-*ThetaD + pi / 2) + (pxy0 - TurningRadius*1000);

					pxx = TurningRadius*cos(-*ThetaD + 3 * pi / 2) * 1000 + pxx0 + TurningRadius * 1000;
					pxy = TurningRadius * 1000 * sin(-*ThetaD + pi / 2) + (pxy0);

					//if (i==3)
				//	cout << i << " iterations have passed by. We sent measurements start moving trajectory" << endl;
				}

			}
			else if (ss == 'o')
			{
				pxx = 900;
				pxy = 400;
			}
			else if (ss == 'g')
			{
				if (i <=2)
				{
					pxx = 400;
					pxy = 640;
				}
				else
				{
					pxx = pxx + (*UD*cos(*ThetaD - atanf(*VD / *UD)) + *VD*(-sin(*ThetaD - atanf(*VD / *UD))))*dts * 1000;
					pxy = pxy + (*VD*cos(*ThetaD - atanf(*VD / *UD)) + *UD*(sin(*ThetaD - atanf(*VD / *UD))))*dts * 1000;
				}

			}
			else if (ss == 'w')
			{
				if (i <=2)
				{
					pxx0 = 300;
					pxy0 = 390;
					pxx = pxx0;
					pxy = pxy0;
				}
				else
				{

					pxx = pxx0 + *UD*currentTime*1000 ;
					pxy = pxy0 + (45 * pi / 180/3.5 * sin(currentTime / 20 * 2 * pi)*1000);

				}

			}
			else
			{
				if (i <=2)
				{
					pxx = 700;
					pxy = 400;

				}
				else
				{
					pxx = pxx + (*UD*cos(*ThetaD) + *VD*(-sin(*ThetaD)))*dts * 1000;
					pxy = pxy + (*VD*cos(*ThetaD) + *UD*(sin(*ThetaD)))*dts * 1000;
				}
			}

			*xp = pxx / 1000; //in m
			*yp = pxy / 1000; // in m  

		
		ostringstream convertphi;   // stream used for the conversion

		std::vector<cv::Point2d> imagePoints;
		std::vector<cv::Point3d> objectPoints;

		objectPoints.push_back(cv::Point3d(pxy, pxx, 0)); //Store Path in List objectPoints

		projectPoints(objectPoints, rvec, tvec, intrinsic, distCoeffs, imagePoints); // Find Path in Image coordinates

		Point path(imagePoints[0].x, imagePoints[0].y); // Define List with the line points in the image

		PXe.push_back(path);

		//Draw the Desired Path 
	//	if (i > 2)
	//	{
	//		for (int i = 0; i < PXe.size() - 1; i++)
	//		{
	//			line(frameROI, PXe[i], PXe[i + 1], Scalar(255, 10, 47), 4);
	//		}
	//	}
		
		// Draw Arrows to point towards desired angle

		Point End((imagePointsState[0].x + abs(*UD - ufi) * 500 + 40 - imagePointsState[0].x)*cos(Theta2D + pi), (imagePointsState[0].y + abs(*UD - ufi) * 500 + 40 - imagePointsState[0].y)*sin(Theta2D + pi));
		Point Start(imagePointsState[0].x + 4, imagePointsState[0].y + 10);
		//arrowedLine(frame1, Start, (Start + End), Scalar(255, 255, 255), 2.5, 8, 0, 0.15);


		// Write the frame into the file '.avi'
		oVideoWriter.write(frame1);


		/////////////////////////////////// POSITION ERROR: ///////////////////////////////////

		*xe = cos(PHI)*(UVPoint.at<double>(1, 0) - pxx) / 1000 + sin(PHI)*(UVPoint.at<double>(0, 0) - pxy) / 1000; // difference between center and Path in world coordinates (m)
		*ye = cos(PHI)*(UVPoint.at<double>(0, 0) - pxy) / 1000 - sin(PHI)*(UVPoint.at<double>(1, 0) - pxx) / 1000; // difference between center and Path in world coordinates (m)
		thetae = PHI1 - (*ThetaD + 0);

		//DISPLAY VIDEO ON WND3

		imshow(wnd3, frameROI);
		imshow("Position/Velocity", frameROI);
		waitKey(1);
		frameROI.release(); //release frame

		char key = cv::waitKey(33);
		if (key == 27)
		{   /* ESC was pressed */
			break;
		}

		clock_end = clock();
		float t = (double)(clock_end - clock_start) / CLOCKS_PER_SEC;
		

		*tim = *tim + t;
		dummyt = *tim;
		if (u <= 2)
		{
			*tim = 0;
		}
		if (u == 2)
		{
			*tim = *tim - dummyt_const;
			dummyt_const = *tim;
		}

		
		
		
		clock_start = clock();

		*xf = UVPoint.at<double>(1, 0);
		*yf = UVPoint.at<double>(0, 0);


		///////////////////////////////////////////////// Save all the states to the matrix mats /////////////////////////////////////////////////

		x0[0] = ufi;
		x0[1] = vfi;
		x0[2] = Omegai;
		x0[3] = *xe;
		x0[4] = *ye;
		x0[5] = PHI;
		x0[6] = *xf;
		x0[7] = *yf;
		x0[8] = PHI1;
		x0[9] = *ThetaD;
		x0[10] = Theta2D;
		x0[11] = *xp; 
		x0[12] = *yp;
		x0[13] = imagePointsState[0].x;
		x0[14] = imagePointsState[0].y;
		x0[15] = ufi2;
		x0[16] = vfi2;
		for (int j = 0; j <= 17; j++)
		{
			if (j == 0)
			{
				if (u < 2)
				{
					mats[u][0] = u;
				}
				
				else
				{
					mats[u][0] = *tim;
				}

				outputO << mats[u][0] << " ";
			}
			else
			{
				mats[u][j] = x0[j - 1];
				outputO << mats[u][j] << " ";
			}
		}

		outputO << "\n";


		////////////////////////// Window Averaging Over sampling window AW (# of measurements) //////////////////////////
		
		
		if (((u+AW-1)>=p && *tim >= sa/3)|| u<=(AW)) 
		{
			for (int l = 0; l < 3; ++l)
			{
				if (u < AW - 1) // If number of iterations < than AW -1 sum from iteration 0 to u 
				{
					for (int n = 0; n <= u; ++n)
					{
						sump[l] = sump[l] + mats[n][l + 1];
					}
					for (int n = 0; n <= u; ++n) 
					{
						sump[3] = sump[3] + (mats[n][9]);
					}

				}
				else  // if number of iterations (measurements) is equal to AW then sum from p to u (this shifts the window)
				{
					for (int n = p; n <= u; ++n)
					{
						sump[l] = sump[l] + mats[n][l + 1];
					}
					for (int n = p; n <= u; ++n)
					{
						sump[3] = sump[3] + (mats[n][9]);
					}
				}
			
			}
	
			if (u < AW - 1)
			{
				*Uf = sump[0] / (u+1);
				*Vf = sump[1] / (u+1);
				*Omegaf = sump[2] / (u+1);
				betaf = sump[3] / (u+1);
			}
			else
			{
				*Uf = sump[0] / (-p + u + 1);
				*Vf = sump[1] / (-p + u + 1);
				*Omegaf = sump[2] / (-p + u + 1);
				betaf = sump[3] / (-p + u + 1);
			}

			x0a[0] = *Uf;
			x0a[1] = *Vf;
			x0a[2] = *Omegaf;
			x0a[3] = PHI1;
			x0a[4] = p;

			for (int j = 0; j <= 8; j++)
			{
				if (j == 0)
				{
					matsa[yy][0] = *tim;
			//		cout << " This is the time stores in matrixoac " << *tim << " at iteration " << u << endl; 
					outputOa << matsa[yy][0] << " ";
				}
				
				else if (j >= 1 && j <= 5)
				{
					if (j == 1){						
					}
					matsa[yy][j] = x0a[j - 1];
				//	cout << x0a[j - 1] << "" << endl; 
					outputOa << matsa[yy][j] << " ";
				}
					
				else if (j == 6)
				{
					matsa[yy][j] = u;
					outputOa << matsa[yy][j] << " ";
				}
				else if (j == 7)
				{
					matsa[yy][j] = ready;
					outputOa << matsa[yy][j] << " ";
				}
			}
			outputOa << "\n";
			q = u;
			yy++;
			//}
			if (u >= (AW - 1))
			    p++;

			sump[0] = 0;
			sump[1] = 0;
			sump[2] = 0;
			sump[3] = 0;

		}

			if ((abs(*alpha_a - alpha_alast) <= control_tolerance) && (abs(*alpha_0 - alpha_0last) <= control_tolerance))
			{
			//CCon = false;
			}

	
	std::unique_lock<std::mutex> lck(M);
		

		if (ready && u>2)
		{
		waitI = t;
		//cout << "COntrol calculation is ready. We got to apply it for " << wait2 << " seconds. Then send back measurements. " << endl;
		av = false; 
 
		alpha_alast = *alpha_a;
		alpha_0last = *alpha_0;
		}
		
		if (u>2)
	//	cout << "This is the " << i << " th iteration. Time in the Sampling loop is " << *tim << endl;


		if (!av )
		{
			if ((CCon == true && u> 2) )
			{
				//cout << "SENDING CONTROLS at time: " << *tim << ":  AMP " << *alpha_a * 180 / pi << " and bias " << *alpha_0 * 180 / pi << endl;

				char  Comd[14];
				char a[] = { '$', 'B' };
				std::copy(a, a + 2, Comd);
				std::copy(cmd, cmd + 11, Comd + 2);
				Comd[13] = { 'X' };

				if (u > 0) // In first iteration, send command. In all rest, first stop tail, then send command. 
				{ 
					unsigned char ar = '$';
					tCom232.SendData(&ar, 1);
					tCom232.ClearSysBuffer();
				}
		
			  for (int i = 0; i < 14; i++) //Sending Control to Fish
				{
					unsigned char ar = Comd[i];
					//cout << Comd[i] << "  " << endl;
					tCom232.SendData(&ar, 1);
				}

			//  CCon = false; 
				
			}
			av = true;
			ready = false; 
			samp = *tim; 
			aT = true;
		}

		//wait2 = 0; 
		if (((((*tim - samp) >= wait2) || ((0.15 <= (0.3 - wait2) && (0.3 - wait2) <= 0.28)) || (((*tim - samp) - wait2) >= -0.15 && (((*tim - samp) - wait2) <= 0))) && aT && u>2) || u == 2) // if wait2 time has passed since controls were obtained send measurements back. ALso if wait time is between 0 and .15 send measurements now. ALso send measurements on the 2 iteration since this is time 0 (we discard the first 3 iterations)
		{

			*xe = *xe;
			*ye = *ye;
			*Beta = x0a[3];//thetae; // 
			*Betaf = PHI1;
			*Uf = x0a[0];
			*Vf = x0a[1];
			*Omegaf = x0a[2];
			*UD = *UD;
			*VD = *VD;
			*tim = *tim;
		//	cout << "These are the states passes to the main thread " << *Uf << *Vf << *Omegaf << "at time :" << *tim << endl; 
			///Same control will be applied as the last time (On the firs iteration) 
			processed = true;
			ready = false;
			lck.unlock();
			CV.notify_one();
			if (u > 2)
			{
				aT = false;
			}
			
		}


		if (*tim > 200)
		{
			*Stop = 1;
			processed = true;
		}

		u = u++;
		i = i++;
	//	CCon = true;
	}

	
	// When everything done, release the video capture and write objects

	cap.release();
	oVideoWriter.release();

	//Send COmmand to straighten the Tail

	char  Comd[14];
	char a[] = { '$', 'B' };
	std::copy(a, a + 2, Comd);
	char end[] = { '1', '0', '0', '.', '0', '0', '0', '0', '.', '0', '0' };
	std::copy(end, end + 11, Comd + 2);
	Comd[13] = { 'X' };

	tCom232.ClearSysBuffer();
	Sleep(50);
	for (int i = 0; i < 14; i++) //Sending Control to Fish
	{
		unsigned char ar = Comd[i];
		tCom232.SendData(&ar, 1);
	}

	cout << "we are leaving Sampling threadh" << endl;
	outputO.close();
	outputOa.close();
	processed = true;
	// Closes all the windows
	destroyAllWindows();
	return 0;
}



//**********INTERFACEBUTTONS**********//
//*********************************//

//tgbutton = drawButton(frameROI, tmsg, Point(310, 0), myPt, CV_RGB(0, 153, 153), CV_RGB(255, 255, 255), tT, 255, 35);
//rectangle(frameROI, Point(600, 10), Point(615, 35), color);


/*if (PHI1 < -pi*1.1*(m)) //Ressetting NM
{
nm = 1;

}

if (-.8*pi*(m + 1) >= mats[i - 1][6] >= -pi*(m + 1) && pi*.9 <= PHI <= pi && nm == 1) // crossed pi line
{
m = m + 1;
nm = nm + 1;
}*/