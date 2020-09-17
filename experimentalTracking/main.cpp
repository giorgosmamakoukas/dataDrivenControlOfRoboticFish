
/*///////////////////////////////////MUST cd('C:\\Users\SML\Documents') in MATLAB ENGINE ONce opened **/

//////////// Include for Operations ////////////////

#include <math.h>
#include <complex>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <iomanip>
#include <string.h>
#include <fstream>
#include <vector>
#include <string>
#include <cstring>
#include <cmath>
#include "omp.h" 
#include "Serial.h"


//////////// Include for Koopman ///////////////

#include "Eigen/Dense"
#include <Eigen/Core>
#include <Eigen/Sparse>
#include "Eigen/Eigen"
#include <Eigen/Eigenvalues>

//////////////// Include for OPENCV Operations ////////////////

#include <matrix.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include "Serial.h"
#include <Winbase.h>
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
#include <vector>

///Headers for other functions 
#include "position.h" // Holds camera/state estimation Function
#include "common.h" //Hold global variables needed for threading 
#include "commonAu.h" //Hold global variables needed for threading 


//////////////// Include for Multi-Threading ////////////////

#include <thread>
#include <condition_variable>
#include <mutex> 

////////////// Include for matlab //////////////

#include "engine.h"
#define  BUFSIZE 256

/////////////////////////////// Declarations ////////////////////////////////

char kp = 'F';//  Online= 'N', OFFline= F

const double pi = 3.14159265358979323846;

// Initiation of control input for XBEE

char comd[11] = { '1', '0', '0', '.', '0', '0', '0', '0', '.', '0', '0' };

// Initial conditions

//FISH Initial Conditions

float Xlive = 0;
float Ylive = 0;
float Blive = 0 * pi / 180;
float Uf = 0.00001785300000000;
float Vf = 0.000001;
float Omegaf = 0.0000747128;

// Trajectory Initial Conditions	
float xp = 0; //%Initial Trajectory x Positions
float yp = 0; //%Initial Trajectory y Positions
float td = 0 * pi / 180;  //%Initial Trajectory heading angle


// Variables for Parallel Programming, control and measurements coordination

std::mutex M; // data lock handle 
std::condition_variable CV;
bool ready = false;
bool processed = false;
int Stop = 0;
float sa = 1.0 / 3.0; //1;//  0.66;//  
double wait2 = sa;
double waitI = 0; 
double TIME = 0.0;


// Variables for Control Projection

float u02 = 0;
float u03 = 0;
double u1 = 0;
double u2 = 0;
double uv1 = 0;
double uv2 = 0;

double alpha_aa = 0.0;

double psi = 0.0;
double psid = 0.0;
double psiddot = 0.0;

using namespace std;
using namespace cv;
using namespace std::chrono;
using namespace Eigen;

float xef = 0;
float yef = 0;
float Theta_e = 0; 

// Desired Trajectory 

extern char ss = 'c'; // Type of Trajectory
float ud = 0.05; // Desired Surge Velocity;
float vd = 0; // Desired Sway Velocity;
float omegad = 0;
float uddot = 0;
float vddot = 0;
float td_ = -pi; // desired heading angle


float R1 = 0.25; // Radius of trajectory
float R2 = R1; // Can be used if doing 8 trajectory
float k1 = 0.15;// angular velocity of trajectory 
float k2 = k1; // other angular velocity. Can be used if doing 8 trajectory

//double omega_a = 3.0*pi;									//%angular velocity of tail fin

float Ra = R1; //This is for other experiments. just leave it
double tim = 0;
double timc = 0;
double ContApply= 0 ; 
float Vc = 0;
float  alphaA = 0;
double f1 = 0;
double f2 = 0;
double f3 = 0;
double xddot = 0;
double xdddot = 0;
double yddot = 0;
double ydddot = 0;
double omegaddot = 0;

double F = 0;
double Fdot = 0;
double alphad = 0;
double alphaddot = 0;
double xedot = 0.0;
double xedot_ = 0.0;
double yedot = 0.0;
double zedot_ = 0.0;

double xe_ = 0.0;
double ze_ = 0.0;

double alpha = 0.0;
double alpha1 = 0;
double alpha2 = 0;
double k_ = 0.0;

double z1 = 0;
double z2 = 0;

double alpha1d = 0;
double alpha2d_ = 0;


float alpha_aa1 = 0;
float alpha_aa2 = 0;
float alpha_aa3 = 0;
float alpha_aa4 = 0;
float alpha_aa5 = 0;
float alpha_aa6 = 0;
float alpha_a0 = 40;
float smallest = 0;
float baa[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

std::complex<double> Yr = 0.0;
std::complex<double> Yr2 = 0.0;
std::complex<double> Yr3 = 0.0;
std::vector<float> Rbaa;


/// Declare functions

int sgn(double x); // sign function that returns +1 if vector is positive and -1 if vector is negative

void psi_x(Eigen::Matrix < double, 6, 1 > & s, Eigen::Matrix < double, 2, 1 > & u, Eigen::Matrix < double, 62, 1 > & psi);

bool compGainMatrix(Eigen::Matrix< double, 62, 62 > &Koopman, Eigen::Matrix< double, 2, 60 > & Kgain);

#define message(x) printf(x "\n\n")
#define BUFSIZE 256

int main()

{
	
	double alpha_0 = 0.0;
	double alpha_a = 0.0;

	if (ss == 'l')
	{
		R1 = 0;
		Ra = R1;
	}
	else if (ss == 'c')
	{
		ud = 0.03;
		//omegad = 0;
		R1 = R1;
	}
	else if (ss == 'i')
	{
		R1 = 0;
		Ra = R1;
		//ud = 0.01;
		//omegad = 0;
	}
	else if (ss == 'v')
	{
		ud = 0.03;
		//omegad = 0;
		R1 = R1;
	}
	sa = sa - 0.3;
	

	//----------------------	---------////////////////////////////////////////// MATRICES/FILES FOR STORING DATA ////////////////////////// ------------------------------- // 
	 
	int NS = 6;   //Number of States 

	// FILES TO STORE STATES/CONTROLS/PREDICTION STATES :

	std::ostringstream filenameB;
	filenameB << "Results//matrixEx" << ss << R1 * 100 << ud * 100 << ".txt";
	string ssmB = filenameB.str();
	ofstream output(ssmB);

	std::ostringstream filenameD;
	filenameD << "Results//matrixPEx" << ss << R1 * 100 << ud * 100 << ".txt";
	string ssmD = filenameD.str();
	ofstream outputp(ssmD);

	//std::ofstream outputC("PlottingNMPCExperiment/control.txt");
	std::ostringstream filenameC;
	filenameC << "Results//controlEx" << ss << R1 * 100 << ud * 100 << ".txt";
	string ssaC = filenameC.str();
	ofstream outputC(ssaC);

	std::ostringstream filenameE;
	filenameE << "Results//BasisEx" << ss << R1 * 100 << ud * 100 << ".txt";
	string ssaE = filenameE.str();
	ofstream outputE(ssaE);

	std::ostringstream filenameEd;
	filenameEd << "Results//BasisdEx" << ss << R1 * 100 << ud * 100 << ".txt";
	string ssaEd = filenameEd.str();
	ofstream outputEd(ssaEd);

	std::ostringstream filenameF;
	filenameF << "Results//LQRGEx" << ss << R1 * 100 << ud * 100 << ".txt";
	string ssaF = filenameF.str();
	ofstream outputF(ssaF);


	// MATRIX TO STORE STATES/CONTROLS/PREDICTION STATES DATA:

	float mat[500][12];
	float matS[500][12];
	float matp[500][5];
	float matc[500][8];
	float matB[500][62];
	float matBd[500][62];

	// Variables to store time 

	time_t clock_start = 0;
	time_t clock_end = 0;
	time_t t2 = 0;
	time_t t2end = 0;
	time_t t22 = 0;
	time_t t2end2 = 0;

	// Vector to store states and control every iteration:

	Eigen::Matrix< double, 2, 1 > Uv = Eigen::Matrix< double, 2, 1 >::Zero();
	Eigen::Matrix< double, 2, 1 > Uvp = Eigen::Matrix< double, 2, 1 >::Zero();
	Eigen::Matrix< double, 6, 1 > P = Eigen::Matrix< double, 6, 1 >::Zero();
	Eigen::Matrix< double, 60, 1 > Psi = Eigen::Matrix< double, 60, 1 >::Zero();
	Eigen::Matrix< double, 60, 1 > Psid = Eigen::Matrix< double, 60, 1 >::Zero();
	Eigen::Matrix< double, 11, 1 > x0 = Eigen::Matrix< double, 11, 1 >::Zero();

	double CTime = 0.0;
	unsigned i = 0;

	duration<double> DT2 ; //Computational Time
	duration<double> DT21; //Computational Time
	duration<double> DT2P; //Computational Time


	/////////////////////LOAD OFFLINE KOOPMAN LQR	/////////////////////


	std::ostringstream filenameCK;
	filenameCK << "OfflineKoopman/LQR"<< ss << R1 * 100 << ud * 100 <<".txt";
	char argCK[32];
	strcpy(argCK, filenameCK.str().c_str());
	char* fileNameCK = argCK;
	ifstream infileCK;
	infileCK.open(fileNameCK);

	//////////////Matrix to store the LQR gains & Basis Functions //////////////

	Eigen::Matrix< double, 2, 60 > Qend = Eigen::Matrix< double, 2, 60 >::Zero();	
	Eigen::Matrix< double, 62, 1 > psi_D = Eigen::Matrix< double, 62, 1 >::Zero();
	Eigen::Matrix< double, 62, 1 > psi_S = Eigen::Matrix< double, 62, 1 >::Zero();
	Eigen::Matrix< double, 6, 1 > s0 = Eigen::Matrix< double, 6, 1 >::Zero();
	Eigen::Matrix< double, 6, 1 > s1 = Eigen::Matrix< double, 6, 1 >::Zero();
	Eigen::Matrix< double, 6, 1 > s2 = Eigen::Matrix< double, 6, 1 >::Zero();
	Eigen::Matrix< double, 6, 1 > s1d = Eigen::Matrix< double, 6, 1 >::Zero();
	Eigen::Matrix< double, 2, 1 > u = Eigen::Matrix< double, 2, 1 >::Zero();



	////////////////////Matlab engine Definitions //////////////
	Engine *ep;
	mxArray *result = NULL;
	char buffer[BUFSIZE + 1];
	mxArray *K_lqr = NULL;
	mxArray *U_ = NULL;
	mxArray *PSI_ = NULL;
	mxArray *PSINEXT_ = NULL;
	mxArray *M_ = NULL;
	mxArray *Param = NULL;

	/*
	* Call engOpen with a NULL string. This starts a MATLAB process
	* on the current host using the command "matlab".
	*/

	if (!(ep = engOpen("")))
	{
		fprintf(stderr, "\nCan't start MATLAB engine\n");
		return EXIT_FAILURE;
	}
	ep = engOpen(NULL);

	engSetVisible(ep, 1); //sets command windo visible 
	
	K_lqr = mxCreateNumericMatrix(2, 60, mxDOUBLE_CLASS,mxREAL);
	U_ = mxCreateNumericMatrix(1, 2, mxDOUBLE_CLASS, mxREAL);
	PSI_ = mxCreateNumericMatrix(9, 1, mxSINGLE_CLASS, mxREAL);
	PSINEXT_ = mxCreateNumericMatrix(9, 1, mxSINGLE_CLASS, mxREAL);
	M_ = mxCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
	Param = mxCreateNumericMatrix(1, 2, mxDOUBLE_CLASS, mxREAL);

	/// C++ arrays used to pass values to MATLAB
	float psi_[9] = {};
	float MM_[1] = {};
	float psinext_[9] = {};
	float up_[2] = {};
	double klqr[120] = {};
	double u_[2] = {};

	//Load KLqr for offline Koopman

		for (int ii = 0; ii < 120; ++ii)
		{		
				infileCK >> klqr[ii];	
					
		}
		int k = 0;
		for (int j = 0; j < 2; ++j)
		{
			for (int ii = 0; ii < 60; ++ii)
			{
				 Qend(j, ii)= klqr[k] ;
				 k++;
			}
		}

	int MM = 2000;
    MM_[0] = MM;

	/// Add the path and cd to where the matlab function files are located
	engEvalString(ep, "addpath('C:\\Users\SML\Documents')");
	//engEvalString(ep, "cd('C:\\Users\SML\Documents')"); // Change directory to find .m function to evaluate basis functions
   // engEvalString(ep, "clear all;"); //clear all variables from MATLAB engine workspace
	
	/// Store M value in Matlab
	memcpy(mxGetPr(M_), MM_, sizeof MM_);
    engPutVariable(ep, "M", M_);

	/// Load offline Koopman
	engEvalString(ep, "learned_model = load('Data_200Hz.mat'); A = learned_model.A; G = learned_model.G;i=1; b=1;k=1;"); // Load A and G the first time
	engEvalString(ep, " Mvec(1)=M; ");
	
	///Define Q and R
	engEvalString(ep, "Q = zeros(60,60);");
	engEvalString(ep, "Q(3,3) = 0.1; Q(4,4) = 1000; R = 1*eye(2);");
	
	// divide by M because Giorgos didn't in the Koopman training
	engEvalString(ep, "A = A/M; G = G/M;"); // Load A and G the first time

	///Start STATE MEASUREMENTS Thread ///
	thread pushTest(position, comd, &xef, &yef, &Theta_e, &Uf, &Vf, &Omegaf, &ud, &vd, &omegad, &uddot, &vddot, &Stop, &td, &xp, &yp, &Xlive, &Ylive, &Blive, &alpha_a, &alpha_0, &TIME);


	//////////////////-------------------------------////////////////////////////////////////// CONTROL MAIN LOOP ////////////////////////// ------------------------------- //////////////////

	double startTime = 0.0;
	double endTime = 40;

	if (ss == 'l')
		endTime = 33.0; // no disturbance endTime = 25.0; // do 35-40 with 
	else if (ss == 'c')
		endTime = 120.0;
	else if (ss == 'w')
		endTime = 25; 

	while (tim< endTime)
	{
		ss = 'c';
		
		//if (ss = 'l')
			//ud = 0.05; 

		///// Declare the name of the lock variable 
		std::unique_lock<std::mutex> lck(M);


		// At iteration 0 Let 3Hz thread have the lock so that it can give us back initial states

		if (i == 0)
		{
			ready = true;
			CV.notify_one();
		}

		// While the other thread is estimating/ obtaining measurements wait.
		while (!processed)
		{		
			CV.wait(lck);
		}

		// Other thread is done and this thread has acquired the right to get data/use it/update control
		// Using global clock from 3Hz thread 
		tim = TIME;
		
		//////////////////////// Storing States //////////////////////// 

		x0(0) = (double)Uf;
		x0(1) = (double)Vf;//0.0023;//
		x0(2) = (double)(Omegaf);//-0.0030;//
		x0(3) = (double)xef;//%Xlive / 1000;//0.0864;// y
		x0(4) = (double)yef;//0.0089;// 
		x0(5) = (double)Theta_e;//-.4239;// 
		x0(6) = (double)Xlive / 1000; //0.0864;// y
		x0(7) = (double)Ylive / 1000; // 0.0089;// 
		x0(8) = (double)Blive;//-.4239;// 
		//x0(9) = DT21.count();

		/////////////////////// Storing Desired Trajectory States //////////////////////// 

		P(0) = ud;
		P(1) = vd;
		P(2) = omegad;
		P(3) = td;
		P(4) = xp;
		P(5) = yp;

		ready = false; //Control values are not ready. 

		//Start timer for determining how long the control calculation takes
		clock_start = clock(); 

		// This is for keepinng a local clock in this thread 
		if (i > 0)
		{
			t2end = clock(); // time elapsed in seconds
			timc = timc + (double)(t2end - t2) / CLOCKS_PER_SEC - waitI;
			t2 = clock();  //Start Timer for time keeping
		}

		// Start this thread's clock 

		if (i==0)
		t2 = clock();  //Start Timer for time keeping


		//////////////////////////////////////////////////////////////// KOOPMAN UPDATE & CONTROL CALCULATION ////////////////////////////////////////////////////////////////
		
		
			/// Copy last KLqr --- either calculated from last iteration or obtained from the offline koopman before the first update

			k = 0;
			for (int j = 0; j < 2; ++j)
			{
				for (int ii = 0; ii < 60; ++ii)
				{
					klqr[k] = Qend(j, ii);
					k++;
				}
			}

			
			/// Don't do any updates until after 2 measurements 
			 if (i >= 1 && kp=='N') // start updating after control has been applied
			 {
				// cout << "Online Koopman so store the last N states" << endl; 
				// After the first 2 measurements we will update s0 and s1 to obtain basis functions
				//s0 << mat[i - 1][7], mat[i - 1][8], mat[i - 1][9], mat[i - 1][1], mat[i - 1][2], mat[i - 1][3];		
				s1 << Xlive / 1000, Ylive / 1000, Blive, Uf, Vf, Omegaf;
				u << Uvp(0), Uvp(1); // Assume constant control over ts window

			   
				//// We are only passin tim,6 states, and control input to matlab. Store thos values in psi_ & psinext_

				/* psi_[0] = mat[i-1][0];
				 for (int j = 1; j <7; ++j)
				 {
					 psi_[j] = s0(j-1);
				 }
				 psi_[7] = u(0);
				 psi_[8] = u(1);
				 */

				 psinext_[0] = tim;
				 for (int j = 1; j < 7; ++j)
				 {
					 psinext_[j] = s1(j-1);
				 }
				 psinext_[7] = u(0); // The controls that take you from state s to s_k+1 are constant. 
				 psinext_[8] = u(1);

				 ////////////////////////// Saves last N states used to calculate discrete Koopman Kd //////////////////////////

				// Copying states_k, states_k+1, M (# of measurements) from C++ to MATLAB variables
				 //memcpy(mxGetPr(PSI_), psi_, sizeof psi_);
				 memcpy(mxGetPr(PSINEXT_), psinext_, sizeof psinext_);
			
				 // Putting MATLAB variables into workspace --- perhaps can be optmiized with above step
				// engPutVariable(ep, "PSI_", PSI_);
				 
				 engPutVariable(ep, "PSINEXT_", PSINEXT_); // Current measurements are store in PSINEXT along with current control and time
				 
	
				//engEvalString(ep, "PSI=psivec(PSI_); PSINEXT=psivec(PSINEXT_); "); //Evaluate basis functions using state measurements

				 /*  Store the basis function measurements and the time */

				 engEvalString(ep, "TempStates(k,:)= PSINEXT_' ; k=k+1;");
			 }
			

			//cout << (timc - CTime) << " has passed since last control was updated" << endl;
			
			if ( (((tim - CTime) >= 0.9 ) || i==0))  // Change control every 1 second  and calculate control in the first iteration 
				{
					ContApply = 1; // use this variable to see when control s applied. SHould be 1 every one second or so 
					CTime = tim; // STore the time the control was applied 

					//high_resolution_clock::time_point  Cstart = high_resolution_clock::now(); // Start a timer to see how long control calulation takes 
				  
					if (kp == 'N'&& i>0)
					{ 
						//cout << "Online Koopman so update LQRs" << endl; 
					////////////// Update LQR after the first iteration. Offline LQR values used in the first one
						
						if (i > 0)
						{

							// Pass to Matlab the old LQR value
							memcpy(mxGetPr(K_lqr), klqr, sizeof klqr);
							engPutVariable(ep, "K_lqr", K_lqr); /// Put lQR variable in workspace

							/* Evaluate matlab function with engEvalString. */

							////////////////////////////////////////////////////* calculate discrete Koopman Kd *////////////////////////////////////////////////////
							
							// Updates KD with measurements // TempStates has constant control calculated since the last control update 
							//Create interpolated data with constant time window separataion ts
							high_resolution_clock::time_point  Cstart = high_resolution_clock::now(); // Start a timer to see how long control calulation takes 
							engEvalString(ep, "tint = TempStates(1,1): 1/200: TempStates(end,1); NN = length(tint);"); // create time to collect measurements at specific times							
							engEvalString(ep, "S_int = interp1([TempStates(:,1)], [TempStates(:, 2:end)], tint, 'spline');"); //interpolate states & control	
							high_resolution_clock::time_point  Cstop = high_resolution_clock::now();

							engEvalString(ep, "Psi= nan(NN,62);"); // Declare Psi_1 & Psi_2 beforehannd; 

							
							engEvalString(ep, "for ii = 1 : NN Psi(ii,:) = psivec([0, S_int(ii,:)]); end"); // add dummy zero for first element in psivec
							
							DT21 = duration_cast<duration<double>>(Cstop - Cstart);// (double)(t2end2 - t22) / CLOCKS_PER_SEC; //sampling rate 


							engEvalString(ep, "Psi_1 = Psi(1:end-1,:); Psi_2 = Psi(2:end,:);"); // shift measurements for Psi_1 and Psi_2 by dt
							// Update A and G matrices to recalculate Kd
							engEvalString(ep, "Mnext = M + NN;"); // increment number of measurements used by NN: Total # of measurements is previous measurements (M) + # of new measurements (NN)
							engEvalString(ep, "A_next = 1/(Mnext) * (A*M + sum(Psi_2'*Psi_1, 3));");
							engEvalString(ep, "G_next = 1/(Mnext) * (G*M + sum(Psi_1'*Psi_1, 3));");
							//engEvalString(ep, "Kd = A_next * pinv(G_next); ");
							engEvalString(ep, " Kd = rescale(A_next) * pinv(rescale(G_next));");
							engEvalString(ep, "Alin = Kd(1:60, 1:60); Blin = Kd(1:60, 60 + 1 : end); "); // CHECK IF and B are correct 

							//don't update them
							// engEvalString(ep, "Kd = A * pinv(G); Alin = Kd(1:60, 1:60); Blin = Kd(1:60, 60 + 1 : end); ");

							////////////////////////////////////////////////////* Calculate LQR values in MATLAB */////////////////////////////////////////////////////

							engEvalString(ep, "Klqr2 = zeros(1, 120); for (n = 1 : 60) Klqr2(2 * n) = K_lqr(2, n); end");
							engEvalString(ep, "for (n=0:59) Klqr2(2*n+1)= K_lqr(1,n+1); end");
							engEvalString(ep, "K_lqr(1,:)= Klqr2(1:60); K_lqr(2,:)= Klqr2(61:120);");

							// Calculate LQR if you can't calculate one use the old one that was passed 
							engEvalString(ep, "try K_lqr = dlqr(Alin, Blin, Q, R); catch warning('KLQR error'); K_lqr; end");
							//engEvalString(ep, "Mvec(i) = M;");
							//engEvalString(ep, "name = [num2str(b), '_iteration']; save(name, 'NN','M','Psi_1','Psi_2','Kd','K_lqr','tint','S_int','Mnext','Alin','Blin', 'A', 'G', 'A_next','G_next','Q','R','Mvec', 'TempStates'); b=b+1; ");
							//engEvalString(ep, "name = [num2str(u), '_iteration']; save(name,'M', 'A', 'G', 'A_next','G_next'); u=u+1; ");

							// Update for next iteration
							engEvalString(ep, "A = A_next; G = G_next; M = Mnext; k=1; clear TempStates;   i=i+1;"); //setting k=1; Everytime you update the control restart the TempStates vector 	

							//engEvalString(ep, "if (abs(K_lqr(1,4) - 25) > 10) save('TESTGiorgos', 'K_lqr'); end ");

							//Use engOutputBuffer to capture  MATLAB output, so we can echo it back.  Ensure first that the buffer is always NULL terminated.*/

							buffer[BUFSIZE] = '\0';
							engOutputBuffer(ep, buffer, BUFSIZE);

							/* Get result of computation*/
							K_lqr = engGetVariable(ep, "K_lqr");

							/* Get Pointed to where result is stored*/
							double *Out = mxGetPr(K_lqr);

							// store lqr obtained from Matlab
							/**/ for (int ii = 0; ii < 60; ++ii)
							{
								for (int j = 0; j < 2; ++j)
								{
									Qend(j, ii) = *Out;
									*Out++;
								}
							}
						   }
						
					}
			
						//Controller 
						// SETTING UP THE FEEDBACK CONTROLLER:
						// -----------------------------------

						s2 << Xlive / 1000, Ylive / 1000, Blive, Uf, Vf, Omegaf;
						s1d << xp, yp, td, ud, vd, omegad;

						// calculate observables
					    psi_x(s1d, u, psi_D);
					    psi_x(s2, u, psi_S);

						// Copy then to a 60x1 Vector
						for (int g = 0; g < 60; g++)
						{
							Psi(g) = psi_S(g);
							Psid(g) = psi_D(g);							
						}

						// Calculate Input

						Uv = -Qend*(Psi - Psid);
						uv1 = Uv(0);
						uv2 = Uv(1);					
				
						Uvp(0) = uv1;
						Uvp(1) = uv2;
			
					   /*high_resolution_clock::time_point  start1 = high_resolution_clock::now();*/
						u_[0] = uv1;//1.09268;// 
						u_[1] = uv2;// 0.114136;//

						memcpy(mxGetPr(U_), u_, sizeof u_);
						engPutVariable(ep, "U_", U_);
						high_resolution_clock::time_point  Prostart = high_resolution_clock::now(); // Start a timer to see how long control calulation takes
						if (i == 0)
						{
							//engEvalString(ep, "rng default");
							engEvalString(ep, "opts = optimoptions(@fmincon, 'Algorithm', 'sqp', 'Display', 'off');");													
						}
						//engEvalString(ep, "name2 = [num2str(b), '_iterationControl']; save(name2,'U_');"); 
						engEvalString(ep,"u1 = U_(1); u2 = U_(2); fun = @(a)(u1 - a(1) ^ 2 * (3 - 3 / 2 * a(2) ^ 2 - 3 / 8 * a(1) ^ 2)) ^ 2 + (u2 - a(1) ^ 2 * a(2)) ^ 2;");
						engEvalString(ep, "problem = createOptimProblem('fmincon', 'objective', fun, 'x0', [0; 0], 'lb', [0; -47 * pi / 180], 'ub', [30 * pi / 180; 47 * pi / 180], 'options', opts); "); 
						engEvalString(ep, "gs = GlobalSearch('Display', 'off','NumStageOnePoints', 100, 'NumTrialPoints', 100, 'FunctionTolerance', 1e-3, 'MaxTime', 0.2);  [x,f] = run(gs,problem); u= x(:)'; Input=u; ");
						//engEvalString(ep, "Input=project_u(U_) "); // Projection in MATLAB
						high_resolution_clock::time_point  ProEnd = high_resolution_clock::now(); // Start a timer to see how long control calulation takes 
						DT2P = duration_cast<duration<double>>(ProEnd - Prostart);// (double)(t2end2 - t22) / CLOCKS_PER_SEC; //sampling rate 
						Param = engGetVariable(ep, "Input");

						/* Get Pointed to where result is stored*/
						double *OutP = mxGetPr(Param);

						alpha_a = *OutP;
						*OutP++;
						alpha_0 = *OutP; 

						
						if (alpha_a == 0 && alpha_0 == 0)
						{
							if (uv2 != 0 && uv2 != -0 && ss == 'c')
								alpha_0 = 47 * pi / 180 * sgn(uv2);
							else
								alpha_0 = 0; 				
						}
			
						Uvp(0) = alpha_a *alpha_a * (3.0 - 3.0 / 2.0 * alpha_0*alpha_0 - 3.0 / 8.0 * alpha_a*alpha_a);
						Uvp(1) = alpha_a* alpha_a * alpha_0;
						t2end2 = clock(); // time elapsed in seconds
				
						

						//DT21 = duration_cast<duration<double>>(Cstop - Cstart);// (double)(t2end2 - t22) / CLOCKS_PER_SEC; //sampling rate 
						
						//cout << "projection takes " << DT21.count() << endl; 
						//DT2 = duration_cast<duration<double>>(stop - start);

						

						
				   }
		
		
				///////////////////////////////////////////////// Construct Control Message /////////////////////////////////////////////////

				std::ostringstream buff1;
				std::ostringstream buff2;

				// Change from radians to degrees

				if (alpha_a > 30*pi/180)
						alpha_a = 30*pi/180;


				if (alpha_0 > 47*pi/180)
					alpha_0 = 47* pi / 180;
				

				if (alpha_0 < -47 * pi / 180)
					alpha_0 = -47 * pi / 180;
			
				double temp1, temp2;
				temp1 = alpha_a * 180 / pi;
				temp2 = alpha_0 * 180 / pi;

		
				buff1 << std::fixed << std::setprecision(2) << temp1;
				buff2 << std::fixed << std::setprecision(2) <<temp2;

				int po = 0;
				int pob = 5;
				int po1 = 0;
				int po1b = 5;
				int fo = 0;

				if (temp1 < 10)
				{
					po = 1;
					pob = 4;
					po1b = 4;
				}

				if (temp2 < 10)
					po1 = 1;

				std::string Am(4, '0');
				std::string B(4, '0');
				std::string DE(1, 'F');

				if (temp2< 0)
				{
					DE = '1';
					fo = 1;
				}

				Am.replace(po, pob, buff1.str());
				B.replace(po1, po1b, buff2.str(), fo, 5);
				strcpy(comd, DE.c_str());
				strcat(comd, Am.c_str());
				strcat(comd, B.c_str());

			//-------------------------------//////////////////////////////////////////  Ready to ask for more data/ Implement controls ////////////////////////// ------------------------------- //

			clock_end = clock(); //End of clock for waiting time 

			if (wait2 < 0)
			{
				wait2 = 0;
			}
			else
			{
				wait2 = sa -  (double)(clock_end - clock_start) / CLOCKS_PER_SEC;// sampling time - control calculation time 
			}
			
			clock_start = clock(); //Start timer for sampling 'waiting'

			
				cout << "at time :" << tim << "the states are "<< x0(0) << x0(1) << x0(2) << endl; 
			// PASS the lock to other function as well as control values so that it can send them to the robot. 

			processed = false;  //Reset Flag that makes Main wait.
			ready = true;     // Update Data
			lck.unlock();
			CV.notify_one(); // Notify thread to write data & Pass the controls 

		//-------------------------------//////////////////////////////////////////  Store all data ////////////////////////// ------------------------------- //

		
		/// Store the states used every 0.3 seconds to update Koopman
		
		for (int j = 0; j <= NS + 4; j++)
			{
				if (j == 0)
				{
					matS[i][0] = tim;
				}
				else
				{
					matS[i][j] = x0(j - 1);
				}
				
			}
		
		//// Store the states used every second for control 
		if (ContApply)
		{

			for (int j = 0; j <= NS + 5; j++)
			{
				if (j == 0)
				{
					mat[i][0] = tim;
					output << mat[i][0] << " ";
				}
				else if (j>0 && j <= NS + 3)
				{
					mat[i][j] = x0(j - 1);
					output << mat[i][j] << " ";
				}

				else if (j==10)
				{
		
					mat[i][j] = DT2P.count();
					output << mat[i][j] << " "; 
					
				}
				else if (j == 11)
				{

					mat[i][j] = DT21.count();
					output << mat[i][j] << " ";
				}

			}
			output << "\n";
		}

	
		if (ContApply)
		{
			for (int j = 0; j <= 6; j++)
			{
				if (j == 0)
				{
					matp[i][0] = tim;
					outputp << matp[i][0] << " ";
				}
				else
				{
					matp[i][j] = P(j - 1);
					outputp << matp[i][j] << " ";
				}
			}
			outputp << "\n";
		}

		/////////////////////// Storing Basis Functions  //////////////////////// 

		for (int j = 0; j <= 60; j++)
		{
			if (j == 0)
			{
				matB[i][0] = tim;
				outputE << matB[i][0] << " ";
			}
			else
			{
				matB[i][j] = Psi(j - 1);
				outputE << matB[i][j] << " ";
			}
		}
		outputE << "\n";

		/////////////////////// Storing Desired Basis Functions  //////////////////////// 

		for (int j = 0; j <= 60; j++)
		{
			if (j == 0)
			{
				matBd[i][0] = tim;
				outputEd << matB[i][0] << " ";
			}
			else
			{
				matBd[i][j] = Psid(j - 1);
				outputEd << matBd[i][j] << " ";
			}
		}
		outputEd << "\n";

		/////////////////////// Storing LQR Gains  //////////////////////// 

		//outputF << i << "\n";

			for (int kk = 0; kk <= 1; kk++)
			{
				for (int j = 0; j <= 59; j++)
				{
					
						outputF << Qend(kk,j) << " ";			
				}
			}
			outputF << "\n";


		//////////////////////// Storing Controls u1-u3 //////////////////////// 
		
			if (ContApply)
			{
				for (int j = 0; j <= 6; j++)
				{
					if (j == 0)
					{
						matc[i][0] = tim;
						outputC << matc[i][0] << " ";
					}

					else if (j >= 1 && j <= 2)
					{

						matc[i][j] = Uv(j - 1);
						outputC << matc[i][j] << " ";
					}

					else if (j >= 3 && j <= 4)
					{

						matc[i][j] = Uvp(j - 3);
						outputC << matc[i][j] << " ";
					}
					else if (j == 5)
					{

						matc[i][j] = alpha_a * 180 / pi;
						outputC << matc[i][j] << " ";
					}
					else if (j == 6)
					{

						matc[i][j] = alpha_0 * 180 / pi;
						outputC << matc[i][j] << " ";
					}

				}

				outputC << "\n";
			}
	
		// end program when time is bigger than endtime 
		if ((tim >= endTime))// || i==6)
		{
			Stop = 1;
		}
		
		ContApply = 0;
		i++; //increment iteration number i 
  }

    cout << "ended" << endl;
	cout << "current time" << tim << endl;
	pushTest.join();
	outputC.close();
	output.close();		
	outputF.close();
	outputEd.close();
	outputE.close();
	outputp.close();
	mxDestroyArray(result);
	mxDestroyArray(K_lqr);
	mxDestroyArray(PSI_);
	mxDestroyArray(PSINEXT_);
	mxDestroyArray(M_);
	engClose(ep);
	return  EXIT_SUCCESS;
}
/// Define functions


int sgn(double x)
{
	return (x > 0) - (x < 0); // returns -1 if x < 0, 1 if x > 0, and 0 if x = 0;
}

void psi_x(Eigen::Matrix < double, 6, 1 > & s, Eigen::Matrix < double, 2, 1 > & u, Eigen::Matrix < double, 62, 1 > & psi)
{

	// Declare states from the first six observables
	double x = s(0); double y = s(1); double th = s(2);
	double v1 = s(3); double v2 = s(4); double omega = s(5);

	double atanv1v2, psi37, psi40, psi52, psi56;
	// To avoid singularities, declare terms that vanish when both linear body-frame velocities are zero.
	if (v2 <= pow(1, -5) && v1 <= pow(1, -5))
	{
		atanv1v2 = 0;
		psi37 = 0;
		psi40 = 0;
		psi52 = 0;
		psi56 = 0;
	}
	else
	{
		atanv1v2 = atan(v2 / v1);
		psi37 = v1 * v2 * v2 * omega / sqrt(v1*v1 + v2*v2);
		psi40 = v1 * v1 * v2  * omega / sqrt(v1*v1 + v2*v2) * atanv1v2;
		psi52 = v1 * v1 * v2 * omega / sqrt(v1*v1 + v2*v2);
		psi56 = v1 * v2 * v2 * omega * atanv1v2 / sqrt(v1*v1 + v2*v2);
	}

	//Declare the observables
	psi(0) = x;
	psi(1) = y;
	psi(2) = th;
	psi(3) = v1;
	psi(4) = v2;
	psi(5) = omega;

	psi(6) = v1 * cos(th) - v2 * sin(th);
	psi(7) = v1 * sin(th) + v2 * cos(th);
	psi(8) = v2 * omega;
	psi(9) = v1 * v1;
	psi(10) = v2 * v2;
	psi(11) = v1 * omega;
	psi(12) = v1 * v2;
	psi(13) = sgn(omega) * omega * omega;

	psi(14) = v2 * omega * cos(th);
	psi(15) = v1 * v1 * cos(th);
	psi(16) = v2 * v2 * cos(th);
	psi(17) = v1 * omega  * sin(th);
	psi(18) = v1 * v2 * sin(th);

	psi(19) = v2 * omega * sin(th);
	psi(20) = v1 * v1 * sin(th);
	psi(21) = v2 * v2 * sin(th);
	psi(22) = v1 * omega * cos(th);
	psi(23) = v1 * v2 * cos(th);

	psi(24) = v1 * omega * omega;
	psi(25) = v1 * v2 * omega;
	psi(26) = v1 * v2 * v2;
	psi(27) = v2 *  sgn(omega)  * omega * omega;
	psi(28) = v1 * v1 * v1;

	psi(29) = v2 * omega * omega;
	psi(30) = v1 * omega * sqrt(v1*v1 + v2 * v2);
	psi(31) = v2 * omega * sqrt(v1*v1 + v2*v2) * atanv1v2;
	psi(32) = v1 * v1 * v2;
	psi(33) = v1 *  sgn(omega) *	 omega * omega;
	psi(34) = v2 * v2 * v2;
	psi(35) = v1 * v1 * v1 * atanv1v2;
	psi(36) = v1 * v2 * v2 * atanv1v2;
	psi(37) = psi37;
	psi(38) = v1 * v1 * v2 * atanv1v2 * atanv1v2;
	psi(39) = v2 * v2 * v2 * atanv1v2 * atanv1v2;
	psi(40) = psi40;

	psi(41) = v2 * v2 * omega;
	psi(42) = v1 * v2 * sqrt(v1*v1 + v2*v2);
	psi(43) = v2 * v2 * sqrt(v1*v1 + v2*v2) * atanv1v2;
	psi(44) = v1 * v1 * omega;
	psi(45) = v1 * v1 * sqrt(v1*v1 + v2*v2) * atanv1v2;
	psi(46) = v1 * v2 *  sgn(omega)  * omega;
	psi(47) = omega * omega * omega;

	psi(48) = v2 * omega * sqrt(v1*v1 + v2*v2);
	psi(49) = v1 *  v1 * v1;
	psi(50) = v1 * v2 * v2;
	psi(51) = v1 * v1 * v2 * atanv1v2;
	psi(52) = psi52;

	psi(53) = v1 * omega * sqrt(v1*v1 + v2*v2) * atanv1v2;
	psi(54) = v1 * v1 * v1 * atanv1v2 *		atanv1v2;
	psi(55) = v1 * v2 * v2 * atanv1v2 * atanv1v2;
	psi(56) = psi56;
	psi(57) = v2 * v2 * v2 * atanv1v2;

	psi(58) = v1 * omega * omega;
	psi(59) = v2 *  sgn(omega) * omega * omega;

	psi(60) = u(0);
	psi(61) = u(1);

}

