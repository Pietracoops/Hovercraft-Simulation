#include"SharedMemory.h"
#include"shared_memory.h"
#include<iostream>
#include<conio.h>
#include <Windows.h>//for sleep

using namespace std;


SharedMemory::SharedMemory(char* Name, int Size, int q, int N)
{
	//char* Name;//Name of Shared Memory
	//int Size;//Size of Shared Memory
	//char* P;//Pointer to Shared Memory
	//int q;//Unique Identifier
	//int N;//total number of programs


	//Initializing native variables to 0
	this->Size = 0;
	this->q = 0;
	this->N = 0;


	double* pdouble;
	int* pint;

	//Making the native variables = to inputs
	this->Size = Size;
	this->q = q;
	this->N = N;



	P = shared_memory(Name, Size);


	if (P == NULL)
	{
		cout << "error" << endl;
		exit(1);
	}

	pdouble = (double*)P;
	pint = (int*)P;




	if (q == 0)//first program in queue
	{
		for (unsigned int i = 0; i < Size / (sizeof(char)); i++)
		{
			P[i] = '0';
		}

		pint[0] = 0;
	}



	cout << "Talking stick at: " << pint[0] << endl;



}



int SharedMemory::write_data(double* A, char* str)
{

	//first thing need to do is check if it is
	//my turn to write into the shared data

	int* pint = (int*)P;
	double* dint = (double*)P;
	int SizeOfString = strlen(str);
	int SizeOfDoubleA = 100;
	int counter;
	int qvalue;




		qvalue = pint[0];
		if (qvalue == q)
		{

			//writing array of doubles
			counter = 0;
			for (int i = 1; i < (1 + SizeOfDoubleA); i++)
			{
				dint[i] = A[counter];
				counter++;
			}

			//writing string
			counter = 0;
			for (int i = SizeOfDoubleA * 8 + 8; i < (8 + SizeOfString + SizeOfDoubleA * 8); i++)
			{
				P[i] = str[counter];
				counter++;
			}


			//increment the q counter
			qvalue++;

			if (qvalue == N)
			{
				qvalue = 0;
			}


			pint[0] = qvalue;

		}
		else
		{
			Sleep(10);
		}

	



	return 0;//1 if everything is good
}



int SharedMemory::read_data(double* A, char* str)
{

	//first thing need to do is check if it is
	//my turn to write into the shared data

	int* pint = (int*)P;
	double* dint = (double*)P;
	int SizeOfString = strlen(str);
	int SizeOfDoubleA = 100;
	int counter;
	int qvalue;

	while (true)
	{
		qvalue = pint[0];
		if (qvalue == q)
		{

			//reading array of doubles
			counter = 0;
			for (int i = 1; i < (1 + SizeOfDoubleA); i++)
			{
				//cout << dint[i] << endl;
				A[counter] = dint[i];
				counter++;
			}


			//reading string
			counter = 0;
			for (int i = SizeOfDoubleA * 8 + 8; i < Size; i++)
			{

				//cout << P[i];
				str[counter] = P[i];
				counter++;
			}
			cout << endl;


			//increment the q counter
			qvalue++;
			break;

		}
		else
		{
			Sleep(10);
		}

	}


	if (qvalue == N)
	{
		qvalue = 0;
	}

	pint[0] = qvalue;
	return 1;//1 if everything is good
}