/************************************************************************************
* Copyright (C) 2016                                                               *
* TETCOS, Bangalore. India                                                         *
*                                                                                  *
* Tetcos owns the intellectual property rights in the Product and its content.     *
* The copying, redistribution, reselling or publication of any or all of the       *
* Product or its content without express prior written consent of Tetcos is        *
* prohibited. Ownership and / or any other right relating to the software and all  *
* intellectual property rights therein shall remain at all times with Tetcos.      *
*                                                                                  *
* Author:    Vishal Bhat
* Date:		 18 September 2020 
*                                                                                  *
* ---------------------------------------------------------------------------------*/

#include "main.h"
#include "Mobility.h"
#include "Animation.h"
double* uavcorr(char* id);
char* matlabloc;

/*init_uav() function is to initialize MATLAB Engine and start UAVs*/
void init_uav()
{
	static int nouav = 1;	// Declared as static, since we want it to be declared and changed only once	
	char buf[100] = "cd ";
	char wp[100];
	char sim[100] = "opSimulink";
	strcpy(wp, matlabloc);
	strcat(buf, wp);	
	if (nouav)					// This will run only at the 1st time
	{

		//MATLAB/SIMULINK INTERFACING
		fprintf(stderr, "\nNetSim is initializing MATLAB Engine process....\n");
		if (!(ep = engOpen(NULL))) {
			MessageBox((HWND)NULL, (LPCWSTR)"Can't start MATLAB engine",(LPCWSTR)"MATLAB_Interface.c", MB_OK);
			fprintf(stderr, "\nMATLAB Initialization Failed\nPress any key to proceed without MATLAB...\n");
			_getch();
		}
		else
		{
			engEvalString(ep, "desktop");
			engEvalString(ep, buf); //Update user-path
			fprintf(stderr, "\nMATLAB initialization completed\n");
			fprintf(stderr, "\nLoading Simulink Model..\n Please press any key after loading UAV!");
			//engEvalString(ep, sim);
			_getch();
		}
		//MATLAB/SIMULINK INTERFACING
		nouav = 0;
	}
}

/*This function is used to receive co-ordinates from MATLAB during run time*/
void uav_run()
{
	MOBILITY_VAR* pstruMobilityVar = (MOBILITY_VAR*)NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId - 1]->pstruDeviceMobility->pstruMobVar;	//Define Mobility variable
	double dPresentTime = pstruMobilityVar->dLastTime;

	memcpy(NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId - 1]->pstruDeviceMobility->pstruCurrentPosition,
		NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId - 1]->pstruDeviceMobility->pstruNextPosition,
		sizeof * NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId - 1]->pstruDeviceMobility->pstruCurrentPosition);

	if (pstruMobilityVar->dLastTime + pstruMobilityVar->dPauseTime * 1000000 < pstruEventDetails->dEventTime + 1000000)	//Everytime Mobility being called
	{
		double* coordinates;		// Pointer for array of X , Y and Z coordinates
		coordinates = uavcorr(pstruEventDetails->nDeviceId);	//Get coordinates from matlab
		if (coordinates != NULL)
		{
			NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId - 1]->pstruDeviceMobility->pstruNextPosition->X = coordinates[0];	// Update the coordinates in Network stack
			NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId - 1]->pstruDeviceMobility->pstruNextPosition->Y = coordinates[1];
			NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId - 1]->pstruDeviceMobility->pstruNextPosition->Z = coordinates[2];
			free(coordinates);			// Free memory of pointer
		}

		//store the last time
		pstruMobilityVar->dLastTime = pstruEventDetails->dEventTime + 100;			// Update Last time since we want to match timings with MATLAB
	}
	//update the device position
	memcpy(NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId - 1]->pstruDevicePosition,
		NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId - 1]->pstruDeviceMobility->pstruCurrentPosition,
		sizeof * NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId - 1]->pstruDevicePosition);

	mobility_pass_position_to_animation(pstruEventDetails->nDeviceId,
		pstruEventDetails->dEventTime,
		DEVICE_POSITION(pstruEventDetails->nDeviceId));

	//Add event for next point 
	pstruEventDetails->dEventTime += (0.01* SECOND);
	fnpAddEvent(pstruEventDetails);
	pstruEventDetails->dEventTime -= (0.01 * SECOND);
}
	
double* uavcorr(int id)
{
	double* coordinates;
	char buf[100];
	mxArray* xmat = NULL;
	mxArray* ymat = NULL;
	mxArray* zmat = NULL;
	double* xcor = NULL;
	double* ycor = NULL;
	double* zcor = NULL;
	if (id == 1)
		engEvalString(ep, "set_param('UAV_Test','SimulationCommand','continue');set_param('UAV_Test','SimulationCommand','pause');");
	
				
	engEvalString(ep, "[xa,c]=size(North)");
	engEvalString(ep, "x_out = North(xa, :)");
	engEvalString(ep, "[ya, c] = size(East)");
	engEvalString(ep, "y_out = East(ya, :)");
	engEvalString(ep, "[za, c] = size(Height)");
	engEvalString(ep, "z_out = Height(za, :)");
	xmat = engGetVariable(ep, "x_out");
	ymat = engGetVariable(ep, "y_out");
	zmat = engGetVariable(ep, "z_out");
	xcor = mxGetPr(xmat);
	ycor = mxGetPr(ymat);
	zcor = mxGetPr(zmat);
	coordinates = (double*)malloc(3 * sizeof * coordinates);
	coordinates[0] = xcor[0];
	coordinates[1] = ycor[0];
	coordinates[2] = zcor[0];
	return (coordinates);
}
