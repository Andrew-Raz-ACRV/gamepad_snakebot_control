// AndrewsLeapAttempt.cpp : Defines the entry point for the console application.
// USER INFormatION IF PROBLEMS OCCUR:
// Turn on PMAC before plugging the ethernet cable into the computer
// Remember to always run Visual studio under Administrator and run Pewin32Pro under Administrator. If PMAC is not connecting at this stage try restarting the computer.
// Plug in the Leap Motion Controller via USB. Check that is on by checking if the red LED lights are on inside it.
// You can check the Leap motion controller quality by running the Leap motion visualizer. If this says it is not connected you may need to reinstall the drivers.

//******************************DIRECT OR DIFFERENTIAL MAPPING CONTROL OF A SNAKE ROBOT PROJECT CODE***************//
// To change to differential, comment the direct mapping file and uncomment the differential mapping file.
// Then comment the lines saying direct mapping

//***************************************
//
//      This is for the Game Controller
//
//		Jeremy Opie 10/10/2019
//
//

//OFFICIAL ORIGINAL SNAKE BOT PROGRAM FOR DEMONSTRATIONS
//OFFICIAL ORIGINAL SNAKE BOT PROGRAM FOR DEMONSTRATIONS
//OFFICIAL ORIGINAL SNAKE BOT PROGRAM FOR DEMONSTRATIONS

//***************INCLUDE FILES***************//
#include "afx.h"
#include "MyPMAC.h"
#include "myRuntime.h"

#include <math.h>

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <string> 
#include <string.h>
//#include "Leap.h"
#include "Gamepad.h"
#include <windows.h>
#include <Eigen/Dense> 

#include <ctime>
#include <stdio.h>
//Having troubles connecting to Eigen after copying this project? Go to the Project Properties and update the additional directories

#define PI 3.14159

using namespace Eigen; //::MatrixXd;

					   //****** RECORDING TO CSV *******//
bool recording = false;
bool forceps_activate = true; //Do not record with the NDI if the forceps are open
bool Direct_mapping = false; //Set this true for direct mapping also switch commenting the include file and find and switch the function below called inverse_Kinematics_code  

//************** PMAC CLASS **************//

class CMyPMAC
{
public:
	void PmacProcess(LPTSTR wParam, LPCTSTR lParam);
	void PmacInit();
	CMyPMAC();
	virtual ~CMyPMAC();

private:
	DWORD dwDevice;
};

CMyPMAC* m_MyPMAC = new CMyPMAC;


//********************************************************
//GLOBAL CONSTANTS REGARDING THE SNAKE ROBOT AND WORKSPACE
//********************************************************

//curvature of tube 2
const float r = 173.58;	//edited by LW 20170308-2:40pm

//conversion to cts
const float radians2counts = 3200 * 50 / 15 / (2 * PI);
const float mm2counts = 400 * 50 / 15;

//tube approximate maximum extrusion in mm (RELATIVE)
const float q1_max = 103000 / mm2counts; //77.25mm
const float q2_max = 110000 / mm2counts; //82.5mm
const float q3_max = 130000 / mm2counts; //97.5mm
const float q6_max = 4 * PI;
const float q6_min = -4 * PI;

//Rotation motor initial angle and distance offsets from origin:
const float q1_offset = 18; //7mm
const float q2_offset = 20; //25mm
const float q3_offset = 28; //30mm
const float q6_offset = 90 * (PI / 180); //degrees to radians
const float q7_offset = 0 * (PI / 180); //degrees to radians
//Tube minimum
const float q1_min = 0; //q1_offset;
const float q2_min = 0; //q2_offset;
const float q3_min = 0; //q3_offset;

const float GCinput_offset = 30; // Offset for game controller inputs to move at a higher resolution


//Rotation and Translation constants
const float z_translation = -550;
const float y_translation = -280;
const float pitch_rotation = 13 * PI / 180; //rotation in degrees to radians //16 was good
const float scale = 2; //Scale	//edited LW 20170803-2:50pm

//WORKSPACE CONSTANTS
const float Z_max = q1_max + q3_max;
//Computing the check points:
const float Z1 = r*sin(q2_max / r); //Z Limits
const float Z2 = Z1 + q3_max * cos(q2_max / r);
const float Z3 = Z2 + q1_max;
const float R_limit1 = r*(1 - cos(q2_max / r)); //R Limits
const float R_limit2 = R_limit1 + q3_max*sin(q2_max / r);
const float M = (R_limit2 - R_limit1) / (Z2 - Z1); //Gradient constants
const float c = R_limit1 - M*Z1;
const float M_m = (R_limit2) / (Z3 - Z_max);
const float c_m = R_limit2 - M*Z3;

//********FILE OF FUNCTIONS REGARDING HAND GESTURE AND WORKSPACE*****//
#include "Hand_Gesture_Workspace_functions.h"

//********FILE OF DIRECT MAPPING INVERSE KINEMATICS*****//
//#include "Direct_Mapping_Inverse_Kinematics_function.h"

//********FILE OF DIFFERENTIAL MAPPING INVERSE KINEMATICS*****//
#include "Differential_Mapping_Inverse_Kinematics_function.h"



//*****************************************************//
//*******************MAIN CODE*************************//
//*****************************************************//
int main()
{
	Gamepad controller(1);

	//Define initial time step
	int time_K = 0;

	//Initialise NDI
	float X_mdi = 0; float Y_mdi = 0; float Z_mdi = 0; float Roll_mdi = 0; float Pitch_mdi = 0; float Yaw_mdi = 0;


	//INITIALISE MAIN VARIABLES
	float X = 0; float Y = 0; float Z = 0;
	float pitch = 0; float yaw = 0; float roll = 0;
	//Leap
	float XL = 0; float YL = 0; float ZL = 0;

	float dX; float dY; float dZ; float dP;
	//ORIGIN RELATIVE CHANGE
	float Xr = 0; float Yr = 0; float Zr = 0;

	//Starting point
	float XL_start = 0; float YL_start = 0; float ZL_start = 0;
	//Previous point 
	float XL_pre = 0; float YL_pre = 0; float ZL_pre = 0;

	//other initialisation
	int k_wind_up = 0; int life = 0;
	bool isopen = false;
	
	bool STOP = false;
	bool Tracking = true; bool Tracking_pre = false;

	//Initialise Current MOTOR VALUES
	float q1c = 0; float q2c = 0; float q3c = 0; float q6c = 0; float q7c = 0;  float q6_pre = 0;
	float Xc = 0; float Yc = 0; float Zc = 0; float Pc = 0; float Rollc = 0; float YAWc = 0;
	float Xcr = 0; float Ycr = 0; float Zcr = 0;
	//******************************************************//
	//----------------initialise PMAC-----------------------//
	//******************************************************//

	TCHAR buf[256];
	CString str, strtemp;
	m_MyPMAC->PmacInit();
	m_MyPMAC->PmacProcess(buf, "i122=64 i222=64 i322=64 i422=64 i522=8 i622=8 i722=8"); //jog speed of axis 1 to axis 6
	m_MyPMAC->PmacProcess(buf, "#1j=0 #2j=0 #3j=0 #4j=0 #5j=0 #6j=0 #7j=0"); // Reset back to it's initial position
	//m_MyPMAC->PmacProcess(buf, "#1j=50000 #2j=100000 #3j=150000 #4j=0 #5j=0 #6j=0 #7j=0");

	//******************************************************//
	// --------------Main Loop of Operation-----------------//
	//******************************************************//
	//Initialise time of loop
	double elapsed_secs = 0;
	
	//SYSTEMTIME st;
	//GetSystemTime(&st);
	//std::cout << "Start time is: " << st.wMinute << " minutes " << st.wSecond << " seconds " << st.wMilliseconds << " miiliseconds\n";
	std::cout << "Welcome to the Game controller Snake Robot Controller\n\n Please enjoy your stay.\n" << std::endl;
	double time = 0;

	m_MyPMAC->PmacProcess(buf, "#1p");
	q1c = round(atof(buf) / mm2counts);

	m_MyPMAC->PmacProcess(buf, "#2p");
	q2c = round((atof(buf) / mm2counts) - q1c);

	m_MyPMAC->PmacProcess(buf, "#3p");
	q3c = round((atof(buf) / mm2counts) - q1c - q2c);

	m_MyPMAC->PmacProcess(buf, "#6p");
	q6c = atof(buf) / radians2counts;

	//adjust by offset
	q1c = q1c + q1_offset;
	q2c = q2c + q2_offset;
	q3c = q3c + q3_offset;
	q6c = q6c + q6_offset;

	//*******************forward kinematics at start of tracking********************//
	Xcr = (q3c*sin(q2c / r) + r*(1 - cos(q2c / r)))*cos(q6c);
	Ycr = (q3c*sin(q2c / r) + r*(1 - cos(q2c / r)))*sin(q6c);
	Zcr = q3c*cos(q2c / r) + r*sin(q2c / r) + q1c;

	
	while (operating(life)) {
		clock_t begin = clock();

		////if the sensor is connected
		if (!controller.Connected()) {
			std::cout << "The Gamepad is not connected. Retrying...\n" << std::endl;
			life++;
			Sleep(10);
			continue;
		}


		////check hand gesture
		//	
		//	std::cout << "correct output: ";
		//	//update readings
		//	//hand angles in radians
		//	float linear_x = !controller.LStick_InDeadzone() ? controller.LeftStick_Y() : 0.0f;
		//	float angular_y = !controller.RStick_InDeadzone() ? controller.RightStick_X() : 0.0f;
		//	float angular_z = !controller.RStick_InDeadzone() ? controller.RightStick_Y() : 0.0f;


		//	//******define relative position*****// 
		//	if ((tracking_pre == false) && (tracking == true)) {
		//		//******************read current motor values*************//
		//		m_MyPMAC->PmacProcess(buf, "#1p");
		//		q1c = round(atof(buf) / mm2counts);

		//		m_MyPMAC->PmacProcess(buf, "#2p");
		//		q2c = round((atof(buf) / mm2counts) - q1c);

		//		m_MyPMAC->PmacProcess(buf, "#3p");
		//		q3c = round((atof(buf) / mm2counts) - q1c - q2c);

		//		m_MyPMAC->PmacProcess(buf, "#6p");
		//		q6c = atof(buf) / radians2counts;

		//		//adjust by offset
		//		q1c = q1c + q1_offset;
		//		q2c = q2c + q2_offset;
		//		q3c = q3c + q3_offset;
		//		q6c = q6c + q6_offset;

		//		//*******************forward kinematics at start of tracking********************//
		//		xcr = (q3c*sin(q2c / r) + r*(1 - cos(q2c / r)))*cos(q6c);
		//		ycr = (q3c*sin(q2c / r) + r*(1 - cos(q2c / r)))*sin(q6c);
		//		zcr = q3c*cos(q2c / r) + r*sin(q2c / r) + q1c;

		//		//record start 
		//		xl_start = xl; yl_start = yl; zl_start = zl;

		//		//define new origin change:
		//		//xr = (x - xc); yr = (y - yc); zr = (z - zc);
		//		//record xl, yl, zl xl_start...


		//		tracking_pre = true;
		//	}
		//		


		//	//rotate and translate readings
		//	//desiredposition desired = rotate2neworigin(x, y, z, pitch, roll, xr, yr, zr);
		//	DesiredPosition desired = rotate2neworigin(xl, yl, zl, pitch, roll, xl_start, yl_start, zl_start, xcr, ycr, zcr, xl_pre,yl_pre,zl_pre);

		//	//update x,y,z based on the desired position
		//	x = desired.x;
		//	y = desired.y;
		//	z = desired.z;
		//	pitch = desired.pitch;
		//	roll = desired.roll;


		//	//user interaction reset life counting
		//	life = 0;
			//check if forceps are closed or not in the correct hand gesture
		
			if (controller.GetButtonPressed(xButtons.R_Shoulder) && !isopen) {
				m_MyPMAC->PmacProcess(buf, "#4j=-3000");
				isopen = true;
				//std::cout << "Open" << std::endl;

			} else if (!controller.GetButtonPressed(xButtons.R_Shoulder) && isopen) {
				m_MyPMAC->PmacProcess(buf, "#4j=10");
				isopen = false;
				//std::cout << "Closed" << std::endl;
			}

			if (controller.GetButtonPressed(xButtons.Start))
			{

				std::cout << "Resetting" << std::endl;
				m_MyPMAC->PmacProcess(buf, "#3j=0");
				m_MyPMAC->PmacProcess(buf, "#2j=0");
				m_MyPMAC->PmacProcess(buf, "#1j=0");
				m_MyPMAC->PmacProcess(buf, "#6j=0");
				m_MyPMAC->PmacProcess(buf, "#7j=0");
				m_MyPMAC->PmacProcess(buf, "#4j=10");
				Sleep(4000);
				std::cout << "Reset!" << std::endl;
			}

			////******************read current motor values*************//
			// LINEAR
			m_MyPMAC->PmacProcess(buf, "#1p");
			q1c = (atof(buf) / mm2counts);
			//
			m_MyPMAC->PmacProcess(buf, "#2p");
			q2c = ((atof(buf) / mm2counts) - q1c);
			//
			m_MyPMAC->PmacProcess(buf, "#3p");
			q3c = ((atof(buf) / mm2counts) - q1c - q2c);
			//
			
			//std::cout << q1c << ", " << q2c << ", " << q3c << ", " << q6c << std::endl;
			////adjust by offset
			q1c = q1c + q1_offset;
			q2c = q2c + q2_offset;
			q3c = q3c + q3_offset;

			// ROTATIONAL
			m_MyPMAC->PmacProcess(buf, "#6p");
			q6c = atof(buf) / radians2counts;
			q6c = q6c + q6_offset;

			m_MyPMAC->PmacProcess(buf, "#7p");
			q7c = atof(buf) / radians2counts;
			q7c = q7c + q7_offset;
			
			////adjust q6 by offset
			//q7c = q7c + q7_offset;
			//
			//
			////std::cout << " current motors: " << " q1 " << q1c << " q2 " << q2c << " q3 " << q3c << " q6 " << q6c << "\n";

			////*******************forward kinematics********************//
			Xc = (q3c*sin(q2c / r) + r*(1 - cos(q2c / r)))*cos(q6c);
			Yc = (q3c*sin(q2c / r) + r*(1 - cos(q2c / r)))*sin(q6c);
			Zc = q3c*cos(q2c / r) + r*sin(q2c / r) + q1c;
			////orientation
			Pc = atan(sin(q6c)*tan(q2c / r));
			YAWc = atan(cos(q6c)*tan(q2c / r));
			Rollc = q7c;

			dX = !controller.RStick_InDeadzone() ? controller.RightStick_X() * -GCinput_offset : 0.0f;
			dY = !controller.RStick_InDeadzone() ? controller.RightStick_Y() *  GCinput_offset : 0.0f;
			dZ = !controller.LStick_InDeadzone() ? controller.LeftStick_Y()  *  GCinput_offset : 0.0f;

			roll = Rollc + controller.RightTrigger() * GCinput_offset - controller.LeftTrigger() * GCinput_offset;

			//if (direct_mapping == false) {
			//	//differential mapping tailoring the differential values
			//	coordinatesxyz coordinates = xyzput_in_workspace(x, y, z);
			//	x = coordinates.x; y = coordinates.y; z = coordinates.z;
			//	dx = x - xc; dy = y - yc; dz = z - zc; dp = pitch - pc;
			//}
			////*// note direct mapping has put in workspace embedded in its algorithm

			////std::cout << " x= " << x << " y= " << y << " z= " << z << " p= " << pitch << "\n";
			////std::cout << " xc= " << xc << " yc= " << yc << " zc= " << zc << " pc= " << pc << "\n";

			////*******************solve inverse kinematics**************//

			////check if the point is in the workspace or if the hand stop moving
			////if ((isinworkspace(x, y, z) == true) && (stop == false)) { //edited by lw 20170308-2:41pm
			//if (stop == false) {

			//	//differential mapping compute motor values
			MotorQ motor_input = Differential_inverse_Kinematics_code(dX, dY, dZ, 0.0, roll, q1c, q2c, q3c, q6c, k_wind_up, isopen);

			//	//direct mapping compute motor values
			//	//motorq motor_input = inverse_kinematics_code(x, y, z, pitch, yaw, roll, q6_pre, q2c, q3c, q1c, k_wind_up, isopen); //consider rounding or not rounding x y z prescision vs noise counts
			//	
			//	
			//	//q6 wind up:
			//	k_wind_up = motor_input.k;

			//	//allocate previous values for next loop
			//	q6_pre = motor_input.q6;

			if (!motor_input.isvalid) {
				continue;
			}

			//		//motor saturation filter: protects tubes by ensuring that tube 1, 2, 3 never goes to negative!
			if (motor_input.q1 < q1_offset) {
				motor_input.q1 = q1_offset;
			}
			if (motor_input.q2 < q2_offset) {
				motor_input.q2 = q2_offset;
			}
			if (motor_input.q3 < q3_offset) {
				motor_input.q3 = q3_offset;
			}

			//		//*******************send the motor commands*************//
			//		/*
			/*std::cout << " give motors: "
			<< " q1 " << motor_input.q1
			<< " q2 " << (motor_input.q2 + motor_input.q1)
			<< " q3 " << (motor_input.q2 + motor_input.q1 + motor_input.q3)
			<< " q4 " << motor_input.q4
			<< " q5 " << motor_input.q5
			<< " q6 " << motor_input.q6
			<< " q7 " << motor_input.q7
			<< "\n";*/
			

			//		//std::cout << " pitch " << (pitch * 180 / pi) << " x: " << x << " y: " << y << " z: " << z;
			//		//std::cout << " yaw " << (yaw * 180 / pi);
			//		std::cout << "\n";

			strtemp.Format("%f", motor_input.q1);
			str = _T("#1j=") + strtemp;
			m_MyPMAC->PmacProcess(buf, str);

			strtemp.Format("%f", (motor_input.q2 + motor_input.q1));
			str = _T("#2j=") + strtemp;
			m_MyPMAC->PmacProcess(buf, str);

			strtemp.Format("%f", (motor_input.q2 + motor_input.q1 + motor_input.q3));
			str = _T("#3j=") + strtemp;
			m_MyPMAC->PmacProcess(buf, str);

			strtemp.Format("%f", motor_input.q6);
			str = _T("#6j=") + strtemp;
			m_MyPMAC->PmacProcess(buf, str);

			strtemp.Format("%f", motor_input.q7); //increase sensitivity of rotation
			str = _T("#7j=") + strtemp;
			m_MyPMAC->PmacProcess(buf, str);

			//	}
			//	else { //if inverse kinematic solution failed
			//		std::cout << "cancelled \n";
			//	}
			//}
			//else { // if outside workspace
			//	std::cout << "outside workspace \n";
			//}
		
		//Record completion time:
		clock_t end = clock();
		elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	}

	//RESET ROBOT
	std::cout << "OPERATION EXPIRED: RESETTING THE SNAKE ROBOT:"
		<< " q1 " << 0
		<< " q2 " << 0
		<< " q3 " << 0
		<< " q4 " << 0
		<< " q5 " << 0
		<< " q6 " << 0
		<< " q7 " << 0
		<< "\n";

	//*******************SEND THE RESET MOTOR COMMANDS*************//
	std::cout << "Program is Terminating now";
	m_MyPMAC->PmacProcess(buf, "#4j=0");
	Sleep(2000); //ensure forceps are closed before going into a tube
				 //Do rotation first:
	m_MyPMAC->PmacProcess(buf, "#6j=0");
	m_MyPMAC->PmacProcess(buf, "#7j=0");
	m_MyPMAC->PmacProcess(buf, "#5j=0");

	bool waiting_for_q6 = true;

	while (waiting_for_q6 == true) {
		m_MyPMAC->PmacProcess(buf, "#6p");
		q6c = atof(buf);
		if (abs(q6c) < 5) {
			//i.e. reading about 0cts in a +-5cts range
			waiting_for_q6 = false;
		}
	}
	//Now the tubes
	return 0;
}


