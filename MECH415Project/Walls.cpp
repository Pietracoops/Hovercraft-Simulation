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

using namespace std;

Walls::Walls(char* Name, int &errorcode)
{

	int n = strlen(Name);

	this->Name = new char[n+1];

	if (this->Name == NULL)
	{
		errorcode = 3;
		return;
	}

	strcpy(this->Name, Name);
	WallMesh = new mesh(this->Name);

	Pos = new double[3];
	Rot = new double[3];


	if (Pos == NULL || WallMesh == NULL || Rot == NULL)
	{
		errorcode = 4;
		return;
	}


	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			MeshExtremities[i][j] = 0.0;
		}
	}

	for (int i = 0; i < 3; i++)
	{
		Pos[i] = 0;
		Rot[i] = 0;
	}

	Length = 10;
	Thickness = 1;
}

Walls::~Walls()
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
	if (Rot != NULL)
	{
		delete Rot;
		Rot = NULL;
	}
	if (WallMesh != NULL)
	{
		delete WallMesh;
		WallMesh = NULL;
	}

}


double & Walls::get_P(int i)
{
	if (i > 2 || i < 0)	exit(1);

	return Pos[i];
}


double & Walls::get_R(int i)
{
	if (i > 2 || i < 0)	exit(1);

	return Rot[i];
}


double & Walls::get_Length()
{

	return Length;
}

double & Walls::get_Width()
{
	return Thickness;
}


void Walls::set_Scale(double Scale)
{
	//Set the Scale of the mesh
	Length *= Scale;
	Thickness *= Scale;


	WallMesh->Scale = Scale;

}

bool & Walls::set_Vertical()
{
	//Set the Scale of the mesh
	
	return Vertical;
}


void Walls::draw()
{
	update_meshextremities();
	WallMesh->draw(Pos[0], Pos[1], Pos[2], Rot[0], Rot[1], Rot[2]);

}



void Walls::update_meshextremities()
{
	static double temp;


	if (Vertical)
	{
		MeshExtremities[0][0] = Pos[0] - Length / 2;       //x - Lowest
		MeshExtremities[0][1] = Pos[0] + Length / 2;       //x - Highest
													       
		MeshExtremities[1][0] = Pos[1] - Thickness / 2;    //y - lowest
		MeshExtremities[1][1] = Pos[1] + Thickness / 2;    //y - highest
	}												       
	else											       
	{												       
		MeshExtremities[0][0] = Pos[0] - Thickness / 2;    //x - Lowest
		MeshExtremities[0][1] = Pos[0] + Thickness / 2;    //x - Highest
													       
		MeshExtremities[1][0] = Pos[1] - Length / 2;       //y - lowest
		MeshExtremities[1][1] = Pos[1] + Length / 2;       //y - highest

	}

}

double & Walls::get_meshextremities(int i, int j)
{

	if (i > 2 || i < 0)	exit(1);
	if (j > 2 || j < 0)	exit(1);

	return MeshExtremities[i][j];
}


void Generate_Maze(Walls* Wall[], int &errorcode)
{
	//Initializing position and meshes of walls

	int counter = 0; // Used to store into array

	for (int j = 0; j < 6; j++)
	{
		Wall[counter] = new Walls("1x10x10_box.X",errorcode);
		Wall[counter]->get_Length() = 10.0;
		Wall[counter]->set_Vertical() = false;
		if (Wall[counter] == NULL || errorcode != 0) return;
		counter++;
	}

	for (int j = 0; j < 4; j++)
	{
		Wall[counter] = new Walls("1x10x20_box.X", errorcode);
		Wall[counter]->get_Length() = 20;
		Wall[counter]->set_Vertical() = false;
		if (Wall[counter] == NULL || errorcode != 0) return;
		counter++;
	}

	for (int j = 0; j < 3; j++)
	{
		Wall[counter] = new Walls("1x10x30_box.X", errorcode);
		Wall[counter]->get_Length() = 30;
		Wall[counter]->set_Vertical() = false;
		if (Wall[counter] == NULL || errorcode != 0) return;
		counter++;
	}

		Wall[counter] = new Walls("1x10x40_box.X", errorcode);
		Wall[counter]->get_Length() = 40;
		Wall[counter]->set_Vertical() = false;
		if (Wall[counter] == NULL || errorcode != 0) return;
		counter++;

		Wall[counter] = new Walls("1x10x50_box.X", errorcode);
		Wall[counter]->get_Length() = 50;
		Wall[counter]->set_Vertical() = false;
		if (Wall[counter] == NULL || errorcode != 0) return;
		counter++;

		Wall[counter] = new Walls("1x10x60_box.X", errorcode);
		Wall[counter]->get_Length() = 60;
		Wall[counter]->set_Vertical() = false;
		if (Wall[counter] == NULL || errorcode != 0) return;
		counter++;

		Wall[counter] = new Walls("1x10x70_box.X", errorcode);
		Wall[counter]->get_Length() = 70;
		Wall[counter]->set_Vertical() = false;
		if (Wall[counter] == NULL || errorcode != 0) return;
		counter++;


	for (int j = 0; j < 3; j++)
	{
		Wall[counter] = new Walls("10x10x1_box.X", errorcode);
		Wall[counter]->get_Length() = 10;
		Wall[counter]->set_Vertical() = true;
		if (Wall[counter] == NULL || errorcode != 0) return;
		counter++;
	}

	for (int j = 0; j < 3; j++)
	{
		Wall[counter] = new Walls("20x10x1_box.X", errorcode);
		Wall[counter]->get_Length() = 20;
		Wall[counter]->set_Vertical() = true;
		if (Wall[counter] == NULL || errorcode != 0) return;
		counter++;
	}


	for (int j = 0; j < 2; j++)
	{
		Wall[counter] = new Walls("30x10x1_box.X", errorcode);
		Wall[counter]->get_Length() = 30;
		Wall[counter]->set_Vertical() = true;
		if (Wall[counter] == NULL || errorcode != 0) return;
		counter++;
	}


		Wall[counter] = new Walls("40x10x1_box.X", errorcode);
		Wall[counter]->get_Length() = 40;
		Wall[counter]->set_Vertical() = true;
		if (Wall[counter] == NULL || errorcode != 0) return;
		counter++;

	for (int j = 0; j < 2; j++)
	{
		Wall[counter] = new Walls("60x10x1_box.X", errorcode);
		Wall[counter]->get_Length() = 60;
		Wall[counter]->set_Vertical() = true;
		if (Wall[counter] == NULL || errorcode != 0) return;
		counter++;
	}

	for (int j = 0; j < 2; j++)
	{
		Wall[counter] = new Walls("100x10x1_box.X", errorcode);
		Wall[counter]->get_Length() = 100;
		Wall[counter]->set_Vertical() = true;
		if (Wall[counter] == NULL || errorcode != 0) return;
		counter++;

	}


	Wall[counter] = new Walls("chest.X", errorcode);
	Wall[counter]->get_Length() = 400;
	Wall[counter]->get_Width() = 200;
	Wall[counter]->set_Scale(0.01);
	Wall[counter]->get_R(2) = PI/2;
	Wall[counter]->get_R(0) = PI;
	Wall[counter]->get_P(2) = 0.5;
	Wall[counter]->set_Vertical() = false;
	if (Wall[counter] == NULL || errorcode != 0) return;

}

void Draw_Maze(Walls* Wall[])
{
	//Drawing Walls at specified location

	int wallcounter = 0;

	//Horizontal - 10
	Wall[wallcounter]->get_P(0) = 55;
	Wall[wallcounter]->get_P(1) = -10;
	Wall[wallcounter]->draw();
	wallcounter++;

	Wall[wallcounter]->get_P(0) = 75;
	Wall[wallcounter]->get_P(1) = -10;
	Wall[wallcounter]->draw();
	wallcounter++;

	Wall[wallcounter]->get_P(0) = 75;
	Wall[wallcounter]->get_P(1) = 20;
	Wall[wallcounter]->draw();
	wallcounter++;

	Wall[wallcounter]->get_P(0) = 85;
	Wall[wallcounter]->get_P(1) = 10;
	Wall[wallcounter]->draw();
	wallcounter++;

	Wall[wallcounter]->get_P(0) = 25;
	Wall[wallcounter]->get_P(1) = 0;
	Wall[wallcounter]->draw();
	wallcounter++;

	Wall[wallcounter]->get_P(0) = 105;
	Wall[wallcounter]->get_P(1) = -50;
	Wall[wallcounter]->draw();
	wallcounter++;


	//Horizontal - 20
	Wall[wallcounter]->get_P(0) = 85;
	Wall[wallcounter]->get_P(1) = -15;
	Wall[wallcounter]->draw();
	wallcounter++;

	Wall[wallcounter]->get_P(0) = 45;
	Wall[wallcounter]->get_P(1) = -15;
	Wall[wallcounter]->draw();
	wallcounter++;

	Wall[wallcounter]->get_P(0) = 55;
	Wall[wallcounter]->get_P(1) = 15;
	Wall[wallcounter]->draw();
	wallcounter++;

	Wall[wallcounter]->get_P(0) = 25;
	Wall[wallcounter]->get_P(1) = -25;
	Wall[wallcounter]->draw();
	wallcounter++;


	//Horizontal - 30
	Wall[wallcounter]->get_P(0) = 65;
	Wall[wallcounter]->get_P(1) = 0;
	Wall[wallcounter]->draw();
	wallcounter++;

	Wall[wallcounter]->get_P(0) = 35;
	Wall[wallcounter]->get_P(1) = -40;
	Wall[wallcounter]->draw();
	wallcounter++;

	Wall[wallcounter]->get_P(0) = 5;
	Wall[wallcounter]->get_P(1) = 10;
	Wall[wallcounter]->draw();
	wallcounter++;

	//Horizontal - 40
	Wall[wallcounter]->get_P(0) = 5;
	Wall[wallcounter]->get_P(1) = -35;
	Wall[wallcounter]->draw();
	wallcounter++;

	//Horizontal - 50
	Wall[wallcounter]->get_P(0) = 95;
	Wall[wallcounter]->get_P(1) = -10;
	Wall[wallcounter]->draw();
	wallcounter++;

	//Horizontal - 60
	Wall[wallcounter]->get_P(0) = 15;
	Wall[wallcounter]->get_P(1) = -15;
	Wall[wallcounter]->draw();
	wallcounter++;

	//Horizontal - 70
	Wall[wallcounter]->get_P(0) = 105;
	Wall[wallcounter]->get_P(1) = -10;
	Wall[wallcounter]->draw();
	wallcounter++;


	//Vertical - 10
	Wall[wallcounter]->get_P(0) = 30;
	Wall[wallcounter]->get_P(1) = -15;
	Wall[wallcounter]->draw();
	wallcounter++;

	Wall[wallcounter]->get_P(0) = 50;
	Wall[wallcounter]->get_P(1) = -15;
	Wall[wallcounter]->draw();
	wallcounter++;

	Wall[wallcounter]->get_P(0) = 80;
	Wall[wallcounter]->get_P(1) = -5;
	Wall[wallcounter]->draw();
	wallcounter++;

	//Vertical -20
	Wall[wallcounter]->get_P(0) = 75;
	Wall[wallcounter]->get_P(1) = 5;
	Wall[wallcounter]->draw();
	wallcounter++;

	Wall[wallcounter]->get_P(0) = 35;
	Wall[wallcounter]->get_P(1) = -5;
	Wall[wallcounter]->draw();
	wallcounter++;

	Wall[wallcounter]->get_P(0) = 25;
	Wall[wallcounter]->get_P(1) = -45;
	Wall[wallcounter]->draw();
	wallcounter++;

	//Vertical -30
	Wall[wallcounter]->get_P(0) = 30;
	Wall[wallcounter]->get_P(1) = 15;
	Wall[wallcounter]->draw();
	wallcounter++;

	Wall[wallcounter]->get_P(0) = 40;
	Wall[wallcounter]->get_P(1) = 5;
	Wall[wallcounter]->draw();
	wallcounter++;


	//Vertical; - 40
	Wall[wallcounter]->get_P(0) = 65;
	Wall[wallcounter]->get_P(1) = -25;
	Wall[wallcounter]->draw();
	wallcounter++;

	//Vertical - 60
	Wall[wallcounter]->get_P(0) = 65;
	Wall[wallcounter]->get_P(1) = -35;
	Wall[wallcounter]->draw();
	wallcounter++;

	Wall[wallcounter]->get_P(0) = 75;
	Wall[wallcounter]->get_P(1) = -45;
	Wall[wallcounter]->draw();
	wallcounter++;


	//Vertical - 100
	Wall[wallcounter]->get_P(0) = 55;
	Wall[wallcounter]->get_P(1) = 25;
	Wall[wallcounter]->draw();
	wallcounter++;

	Wall[wallcounter]->get_P(0) = 55;
	Wall[wallcounter]->get_P(1) = -55;
	Wall[wallcounter]->draw();
	wallcounter++;


	//Chest
	Wall[wallcounter]->get_P(0) = 100;
	Wall[wallcounter]->get_P(1) = -50;
	//Wall[wallcounter]->get_P(0) = -20;//Setting the chest directly behind start position for testing purposes
	//Wall[wallcounter]->get_P(1) = 0;
	Wall[wallcounter]->draw();

}

