#include <iostream>  
#include "Objects.h"
#include <cmath>   
#include <cstdio>  
#include <cstring> 
#include <fstream>   
#include <strstream> 
#include <iomanip>   
#include <windows.h> 
#include "timer.h"  
#include "rotation.h" 
#include "ran.h"
#include <mmsystem.h>

using namespace std;

Vehicle::Vehicle(char* Name, int &errorcode)
{

	int n = strlen(Name);

	this->Name = new char[n+1];

	if (this->Name == NULL)
	{
		errorcode = 1;
		return;
	}

	strcpy(this->Name, Name);
	VehicleMesh = new mesh(this->Name);

	Pos = new double[3];
	Vel = new double[3];
	Rot = new double[3];

	if (Pos == NULL || Vel == NULL|| VehicleMesh == NULL || Rot == NULL)
	{
		errorcode = 2;
		return;
	}


	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			MeshExtremities[i][j] = 0.0;
		}
	}



	for (int i = 0; i < 3; i++)
	{
		Pos[i] = 0;
		Vel[i] = 0;
		Rot[i] = 0;
	}

	Length = 750;
	Width = 350;
	theta = 0;
	friction = 1;
	dforce = 0.03;
	FRight = 0;
	FLeft = 0;
	Pos[2] = 0.9;

	t0 = high_resolution_time();
	t_sim = 0.0;
}

Vehicle::~Vehicle()
{

	if (Name != NULL)
	{
		delete Name;
		Name = NULL;
	}
	if (Pos != NULL)
	{
		delete Pos;
		Pos = NULL;
	}
	if (Vel != NULL)
	{
		delete Vel;
		Vel = NULL;
	}
	if (Rot != NULL)
	{
		delete Rot;
		Rot = NULL;
	}
	if (VehicleMesh != NULL)
	{
		delete VehicleMesh;
		VehicleMesh = NULL;
	}

}


double & Vehicle::get_P(int i)
{
	if (i > 2 || i < 0)	exit(1);

	return Pos[i];
}

double & Vehicle::get_V(int i)
{
	if (i > 2 || i < 0)	exit(1);

	return Vel[i];
}

double & Vehicle::get_R(int i)
{
	if (i > 2 || i < 0)		exit(1);

	return Rot[i];
}


double & Vehicle::get_Length()
{
	
	return Length;
}


double & Vehicle::get_Width()
{

	return Width;
}

void Vehicle::set_Scale(double Scale)
{
	//Set the Scale of the mesh
	Length *= Scale;
	Width *= Scale;

	VehicleMesh->Scale = Scale;

}

double Vehicle::get_theta()
{
	return theta;
}

void Vehicle::get_highscore(int &errorcode)
{
	static char file_name[50] = "highscore.txt";
	static ifstream fin;
	static ofstream fout;

	fin.open(file_name);

	// if file doesn't exist start a new count
	if (!fin) {
		//file doesn't exist create file for first time
		highscore = 9.99E300;
		fout.open(file_name);
		if (!fout) errorcode = 5;
		fout << highscore;
		fout.close();
	}
	else {
		fin >> highscore;
	}
	fin.close();
}

void Vehicle::store_highscore(double highscore, double Restart, int &errorcode)
{
	static char file_name[50] = "highscore.txt";
	static ofstream fout;

	if (Restart == 2)//Pickup chest
	{
		fout.open(file_name);

		if (!fout){
			errorcode = 6;
		}
		else {
			fout << highscore;
		}
		fout.close();

	}


}


void Vehicle::draw()
{
	update_meshextremities();
	VehicleMesh->draw(Pos[0], Pos[1], Pos[2], Rot[0], Rot[1], Rot[2]);

}

double & Vehicle::get_friction()
{
	return friction;
}


void Vehicle::set_music(char* wavfile)
{

	PlaySound(TEXT(wavfile), NULL, SND_FILENAME | SND_LOOP | SND_ASYNC);
}


void Vehicle::sim_step(double dt,double t, int &restart)
{

	if (restart == 1 || restart == 2)
	{
		t_sim = 0;
	}

	if (t - t_sim >= dt)
	{	

		if (KEY(VK_LEFT))
		{
			if (FRight < 50) FRight += 4*dforce;//increase
			if (FLeft > 0) FLeft -= friction*dforce;//decreasing the power of the other one
			if (FLeft < 0) FLeft = 0;//do not exceed 0
		}
		else if (KEY(VK_RIGHT))
		{
			if (FLeft < 50) FLeft += 4 * dforce;
			if (FRight > 0) FRight -= friction*dforce;
			if (FRight < 0) FRight = 0;
		}
		else if (KEY(VK_UP))
		{
			//Both engines at same time
			if (FRight <= FLeft)
				if (FRight < 50) FRight += 4 * dforce;
			
			if (FLeft <= FRight)
				if (FLeft < 50) FLeft += 4 * dforce;
		}
		else
		{
			//Decrement until reaches 0
			if (FRight > 0)	FRight -= friction*dforce;
			if (FLeft > 0)	FLeft -= friction*dforce;
			if (FRight < 0) FRight = 0;
			if (FLeft < 0) FLeft = 0;
		}


		Euler(FLeft, FRight, dt);//Call Euler Functions
		
		t_sim += dt;//Increment simulation time
	}

}

void Vehicle::Euler(double F1, double F2, double dt)
{

	static double c1 = 0.0, c2 = 0.0, c3 = 0.0;
	static double M = 0.085;//grams
	static double u = 0, v = 0, udot = 0, vdot = 0;
	static double deltatheta = 0, rdot = 0, r = 0;
	static double J = 0.05;//moment of inertia (kg*m^2)
	static double theta = 0.0;
	static double sim_time = 0;
	static double length = 5;


		udot = ((F1 + F2 - c1*u) / M) + v*r;
		vdot = ((-c2*v) / M) - u*r;
		rdot = ((length / 2)*(F1 - F2) - c3*r) / J;

		u = udot*dt;
		v = vdot*dt;
		r = rdot*dt;

		//Conversion to Global Coordinates
		theta += r*dt;
		Vel[0] = cos(theta)*u - sin(theta)*v;
		Vel[1] = sin(theta)*u + cos(theta)*v;

		this->theta = theta;

		//Velocity to Position
		Pos[0] += Vel[0]*dt;
		Pos[1] += Vel[1]*dt;

}




void Vehicle::set_camera_view(int &i)
{
	static double time0 = high_resolution_time();
	static double timenow;
	static double stalltime = 0.5;
	static double LastPressTime;
	timenow = high_resolution_time() - time0;

	if (timenow - LastPressTime > stalltime)
	{
		if (KEY('C'))
		{
			i++;
			LastPressTime = high_resolution_time() - time0;
		}
	}


	if (i > 3) i = 1;//total of 2 camera views

	if (i == 1)
	{
		//first person

		eye_point_g[1] = Pos[0];
		eye_point_g[2] = Pos[1];
		eye_point_g[3] = 0.9;

		lookat_point_g[1] = (10 * cos(theta)) + Pos[0];
		lookat_point_g[2] = (10 * sin(theta)) + Pos[1];
		lookat_point_g[3] = 0.0;

		up_dir_g[1] = 0.0;
		up_dir_g[2] = 0.0;
		up_dir_g[3] = 1.5;

		set_view(eye_point_g, lookat_point_g, up_dir_g);
	}
	else if (i == 2)
	{
		//3rd person


		eye_point_g[1] = Pos[0] + (-6 * cos(-theta));
		eye_point_g[2] = Pos[1] + (6 * sin(-theta));
		eye_point_g[3] = 3;

		lookat_point_g[1] = 0.0 + Pos[0];
		lookat_point_g[2] = 0.0 + Pos[1];
		lookat_point_g[3] = 2.0;
		
		up_dir_g[1] = 0.0;
		up_dir_g[2] = 0.0;
		up_dir_g[3] = 1.0;


		set_view(eye_point_g, lookat_point_g, up_dir_g);
	}
	else if (i == 3)
	{
		//controllable view to see top view of maze
		//Q = +X    |   A = -X
		//W = +Y    |   S = -Y
		//E = +Z    |   D = -Z
		//R = +Dia  |   F = -Dia
		set_view();
	}

}

void Vehicle::update_meshextremities()
{

	//Front of Hovercraft
	MeshExtremities[0][0] = Pos[0] + (Length / 2)*cos(theta);
	MeshExtremities[0][1] = Pos[1] + (Length / 2)*sin(theta);	

	//Top Right Hovercraft
	MeshExtremities[1][0] = Pos[0] + (Length / 2 - (Length* 0.3733))*sin(theta);
	MeshExtremities[1][1] = Pos[1] - (((0.8*Width) / 2))*cos(theta);

	//Bottom Right Hovercraft
	MeshExtremities[2][0] = Pos[0] - sqrt(((Length / 2)*(Length / 2) + (Width / 2)*(Width / 2)))*cos(theta + 2*PI/14.4);
	MeshExtremities[2][1] = Pos[1] - sqrt(((Length / 2)*(Length / 2) + (Width / 2)*(Width / 2)))*sin(theta + 2*PI / 14.4);

	//Bottom Left Hovercraft
	MeshExtremities[3][0] = Pos[0] - sqrt(((Length / 2)*(Length / 2) + (Width / 2)*(Width / 2)))*sin((theta + 2*PI / 5.538));
	MeshExtremities[3][1] = Pos[1] + sqrt(((Length / 2)*(Length / 2) + (Width / 2)*(Width / 2)))*cos((theta + 2 * PI / 5.538));

	//Top Left Hovercraft
	MeshExtremities[4][0] = Pos[0] + (Length / 2 - (Length* 0.3733))*sin(-theta);
	MeshExtremities[4][1] = Pos[1] + (((0.8*Width) / 2))*cos(-theta);

}

double & Vehicle::get_meshextremities(int i, int j)
{

	if (i > 4 || i < 0) exit(1);
	if (j > 2 || j < 0) exit(1);

	return MeshExtremities[i][j];
}


int collision(Vehicle & Hovercraft, Walls* Wall[],int NumberOfWalls)
{


	for (int i = 0; i < NumberOfWalls - 1; i++)
	{
		

		//Point 1 
		if (Hovercraft.get_meshextremities(0, 0) > Wall[i]->get_meshextremities(0, 0) && Hovercraft.get_meshextremities(0, 0) < Wall[i]->get_meshextremities(0, 1) && \
			Hovercraft.get_meshextremities(0, 1) > Wall[i]->get_meshextremities(1, 0) && Hovercraft.get_meshextremities(0, 1) < Wall[i]->get_meshextremities(1, 1))
		{
			//Point 1 is within restricted area
			Hovercraft.FLeft = 0;
			Hovercraft.FRight = 0;
			

			Hovercraft.get_P(0) = 0;
			Hovercraft.get_P(1) = 0;
			return 1;
			
		}

		
		//Point 2 
		if (Hovercraft.get_meshextremities(1, 0) > Wall[i]->get_meshextremities(0, 0) && Hovercraft.get_meshextremities(1, 0) < Wall[i]->get_meshextremities(0, 1) && \
			Hovercraft.get_meshextremities(1, 1) > Wall[i]->get_meshextremities(1, 0) && Hovercraft.get_meshextremities(1, 1) < Wall[i]->get_meshextremities(1, 1))
		{
			//Point 2 is within restricted area
			Hovercraft.FLeft = 0;
			Hovercraft.FRight = 0;

			Hovercraft.get_P(0) = 0;
			Hovercraft.get_P(1) = 0;
			return 1;
		}


		//Point 3 
		if (Hovercraft.get_meshextremities(2, 0) > Wall[i]->get_meshextremities(0, 0) && Hovercraft.get_meshextremities(2, 0) < Wall[i]->get_meshextremities(0, 1) && \
			Hovercraft.get_meshextremities(2, 1) > Wall[i]->get_meshextremities(1, 0) && Hovercraft.get_meshextremities(2, 1) < Wall[i]->get_meshextremities(1, 1))
		{
			//Point 3 is within restricted area
			Hovercraft.FLeft = 0;
			Hovercraft.FRight = 0;

			Hovercraft.get_P(0) = 0;
			Hovercraft.get_P(1) = 0;
			return 1;
		}


		//Point 4 
		if (Hovercraft.get_meshextremities(3, 0) > Wall[i]->get_meshextremities(0, 0) && Hovercraft.get_meshextremities(3, 0) < Wall[i]->get_meshextremities(0, 1) && \
			Hovercraft.get_meshextremities(3, 1) > Wall[i]->get_meshextremities(1, 0) && Hovercraft.get_meshextremities(3, 1) < Wall[i]->get_meshextremities(1, 1))
		{
			//Point 4 is within restricted area
			Hovercraft.FLeft = 0;
			Hovercraft.FRight = 0;

			Hovercraft.get_P(0) = 0;
			Hovercraft.get_P(1) = 0;
			return 1;
		}

		//Point 5 
		if (Hovercraft.get_meshextremities(4, 0) > Wall[i]->get_meshextremities(0, 0) && Hovercraft.get_meshextremities(4, 0) < Wall[i]->get_meshextremities(0, 1) && \
			Hovercraft.get_meshextremities(4, 1) > Wall[i]->get_meshextremities(1, 0) && Hovercraft.get_meshextremities(4, 1) < Wall[i]->get_meshextremities(1, 1))
		{
			//Point 5 is within restricted area
			Hovercraft.FLeft = 0;
			Hovercraft.FRight = 0;

			Hovercraft.get_P(0) = 0;
			Hovercraft.get_P(1) = 0;
			return 1;
		}

	}

	//CHECKING FOR CHEST

	//Point 1 
	if (Hovercraft.get_meshextremities(0, 0) > Wall[30]->get_meshextremities(0, 0) && Hovercraft.get_meshextremities(0, 0) < Wall[30]->get_meshextremities(0, 1) && \
		Hovercraft.get_meshextremities(0, 1) > Wall[30]->get_meshextremities(1, 0) && Hovercraft.get_meshextremities(0, 1) < Wall[30]->get_meshextremities(1, 1))
	{
		//Point 1 is within restricted area
		Hovercraft.FLeft = 0;
		Hovercraft.FRight = 0;

		Hovercraft.get_P(0) = 0;
		Hovercraft.get_P(1) = 0;

		return 2;

	}


	//Point 2 
	if (Hovercraft.get_meshextremities(1, 0) > Wall[30]->get_meshextremities(0, 0) && Hovercraft.get_meshextremities(1, 0) < Wall[30]->get_meshextremities(0, 1) && \
		Hovercraft.get_meshextremities(1, 1) > Wall[30]->get_meshextremities(1, 0) && Hovercraft.get_meshextremities(1, 1) < Wall[30]->get_meshextremities(1, 1))
	{
		//Point 2 is within restricted area
		Hovercraft.FLeft = 0;
		Hovercraft.FRight = 0;

		Hovercraft.get_P(0) = 0;
		Hovercraft.get_P(1) = 0;

		return 2;
	}


	//Point 3 
	if (Hovercraft.get_meshextremities(2, 0) > Wall[30]->get_meshextremities(0, 0) && Hovercraft.get_meshextremities(2, 0) < Wall[30]->get_meshextremities(0, 1) && \
		Hovercraft.get_meshextremities(2, 1) > Wall[30]->get_meshextremities(1, 0) && Hovercraft.get_meshextremities(2, 1) < Wall[30]->get_meshextremities(1, 1))
	{
		//Point 3 is within restricted area
		Hovercraft.FLeft = 0;
		Hovercraft.FRight = 0;

		Hovercraft.get_P(0) = 0;
		Hovercraft.get_P(1) = 0;

		return 2;
	}


	//Point 4 
	if (Hovercraft.get_meshextremities(3, 0) > Wall[30]->get_meshextremities(0, 0) && Hovercraft.get_meshextremities(3, 0) < Wall[30]->get_meshextremities(0, 1) && \
		Hovercraft.get_meshextremities(3, 1) > Wall[30]->get_meshextremities(1, 0) && Hovercraft.get_meshextremities(3, 1) < Wall[30]->get_meshextremities(1, 1))
	{
		//Point 4 is within restricted area
		Hovercraft.FLeft = 0;
		Hovercraft.FRight = 0;

		Hovercraft.get_P(0) = 0;
		Hovercraft.get_P(1) = 0;

		return 2;
	}

	//Point 5 
	if (Hovercraft.get_meshextremities(4, 0) > Wall[30]->get_meshextremities(0, 0) && Hovercraft.get_meshextremities(4, 0) < Wall[30]->get_meshextremities(0, 1) && \
		Hovercraft.get_meshextremities(4, 1) > Wall[30]->get_meshextremities(1, 0) && Hovercraft.get_meshextremities(4, 1) < Wall[30]->get_meshextremities(1, 1))
	{
		//Point 5 is within restricted area
		Hovercraft.FLeft = 0;
		Hovercraft.FRight = 0;

		Hovercraft.get_P(0) = 0;
		Hovercraft.get_P(1) = 0;

		return 2;
	}



	return 0;
}


void Display_Instructions(double t, int &Restart, double &tfinal)
{

	//Restart 0 = No Collision  |  Restart 1 = Wall Collision  |  Restart 3 = Chest Collision
	if (Restart == 1 || Restart == 0)
	{
		if (t < 5.0)
		{
			text_xy("Complete the Maze as fast as possible!", 650, 500, 30);
		}
		if (t > 5.0 && t < 10.0)
		{
			text_xy("Don't touch the walls or you will have to restart", 650, 500, 30);
		}

		text_xy("Best Time:" , 900, 100, 20);

		if (tfinal == 9.99E300)
		{
			text_xy(tfinal, 900, 150, 20);
		}
		else
		{
			text_xy(tfinal, 900, 150, 20);
		}
		
		Restart = 0;
	}
	if (Restart == 2)
	{
		if (tfinal == 9.99E300)
		{
			text_xy(0.0, 900, 150, 20);
		}
		else
		{
			text_xy(tfinal, 900, 150, 20);
		}

		Restart = 0;
	}

	text_xy(t, 100, 100, 30);   // Display Time
}

void Display_Error(int ErrorCode)
{

	//Displaying Error Message to Screen

	if (ErrorCode == 1)
	{
		text_xy("Dynamic Memory error for Vehicle Name", 100, 900, 30);
	}
	else if (ErrorCode == 2)
	{
		text_xy("Dynamic Memory error for Vehicle Position, Velocity, Rotation, and/or Mesh", 100, 900, 30);
	}
	else if (ErrorCode == 3)
	{
		text_xy("Dynamic Memory error for Wall Name", 100, 900, 30);
	}
	else if (ErrorCode == 4)
	{
		text_xy("Dynamic Memory error for Wall Position, Velocity, Rotation, and/or Mesh", 100, 900, 30);
	}
	else if (ErrorCode == 5)
	{
		text_xy("Get_highscore: Unable to create highscore databse - FileIO error", 100, 900, 30);
	}
	else if (ErrorCode == 6)
	{
		text_xy("Store_highscore: Unable to store highscore into databse - FileIO error", 100, 900, 30);
	}

}
