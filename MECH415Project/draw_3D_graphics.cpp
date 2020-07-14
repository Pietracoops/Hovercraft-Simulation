#include <cmath>           // math functions
#include <cstdio>          // standard I/O functions
#include <cstring>         // string manipulation functions
#include <iostream>        // console stream I/O
#include <fstream>         // file stream I/O
#include <strstream>       // string stream I/0
#include <iomanip>         // I/O manipulators
#include <windows.h>       // for keyboard input
#include "math.h"	       // more math functions
#include "Objects.h"       // Vehicle and maze functions
#include "timer.h"         // for measuring time
#include "rotation.h"      // for computing rotation functions
#include "shared_memory.h" // Mech415 shared mem class 
#include "SharedMemory.h"  // Custom SharedMemory class


using namespace std;


//**********FUNCTION DECLARATIONS**********************************
int collision(Vehicle & Hovercraft, Walls* Wall[], int NumberOfWalls); // Collision detection
void Draw_Maze(Walls* Wall[]);										   // Draw the maze
void Generate_Maze(Walls* Wall[],int &errorcode);					   // Set position of maze walls
void Display_Instructions(double t, int &Restart, double &tfinal);	   // Show instructions at beginning of game
void Display_Error(int ErrorCode);									   // Display Error messages
//*****************************************************************



int WIDTH_MIN = 0;
int HEIGHT_MIN = 0;
int WIDTH_MAX = GetSystemMetrics(SM_CXSCREEN);//getting screen resolution X
int HEIGHT_MAX = GetSystemMetrics(SM_CYSCREEN);//getting screen resolution Y

// background colour for the scene
float BACK_R = 0.0f; //colour component (0 to 1)
float BACK_G = 0.0f; 
float BACK_B = 0.5f; 

double VMIN = 0.25; 
double VMAX = 1000.0; 



void draw_3D_graphics() 

{
	static const int N = 31;   // Number of walls
	static const int n = 100;  // size of shared memory block (bytes)
	static int errorcode = 0;  // Error code flag
	static int init = 0;	   // initialization flag
	static int CameraView;     // Setting first person / Third person view
	static int Restart;	       // Restart Flag (1 = Collision with Wall | 2 = Pickup Chest | 3 = original set_view() )


	static double dt = 0.03;   // Delta Time
	static double t;	       // clock time from t0
	static double t0;	       // initial clock time
	static double tfinal;	   // Player Final Completion Time
	static double BestTime;    // Player Best Time
	static char trackname[50] = "random2.wav";
	static char unit[8] = "seconds";

	static Vehicle Massimo("Assets\\hovercraft\\BHoverCraft10.x",errorcode);  // Hovercraft
	static Walls *Anthony[N];									              // Walls
	static mesh terrain("terrain.X");								          // Background
	static SharedMemory SM("shared_memory1", n, 0, 2);			              // Block of shared memory (size = n, 0 = queue number, 2 = number of programs in queue (from 0 to 1))
	
	//**********INITIALIZATION*************************************************
	if (!init) {
		init = 1;

		Restart = 0;
		t0 = high_resolution_time(); // initial clock time (s)
		tfinal = 0;
		CameraView = 1;              //Set Camera to First Person

		Generate_Maze(Anthony,errorcode);

		Massimo.get_highscore(errorcode);
		BestTime = Massimo.highscore;
		Massimo.set_music(trackname);
		Massimo.get_R(0) = PI / 2;
		Massimo.get_R(1) = 0;
		Massimo.get_R(2) = PI / 2;
		Massimo.set_Scale(0.004);
	} 
	//*************************************************************************


	t = high_resolution_time() - t0;                        //Total sim duration
	
	SM.write_data(&t, unit);                                //Write time to shared memory

	Massimo.store_highscore(BestTime, Restart, errorcode);  //Store highscore

	Massimo.get_R(0) = (3.14159 / 2) + Massimo.get_theta();	//rotating mesh

	Massimo.sim_step(dt, t, Restart);                       //Updating with Eulers

	Massimo.set_camera_view(CameraView);

	
	//draw_XYZ(100);                                        //set axes of 100 m length


	//********* Drawing *********************************************************
	terrain.draw(0, 0, 0, 0, 0, 0);                                           //*
	Draw_Maze(Anthony);							                              //*
	Massimo.draw();								                              //*
	Display_Instructions(t, Restart, BestTime);                               //*
	Display_Error(errorcode);												  //*
	//***************************************************************************


	if (collision(Massimo, Anthony, N) == 1)                //Check for collision
	{
		t0 = high_resolution_time();
		Restart = 1;
	}
	else if (collision(Massimo, Anthony, N) == 2)
	{
		tfinal = t;
		if (tfinal < BestTime) BestTime = tfinal;
		t0 = high_resolution_time();
		Restart = 2;
	}
	
}

