
#include <iostream>
#include <conio.h>
#include <Windows.h>

#include "shared_memory.h"
#include "SharedMemory.h"

using namespace std;

int main(int argc, char* argv[])
{
	//int i;
	int n = 100; // size of shared memory block (bytes)
	char name[] = "shared_memory1"; // name of shared memory block

	double A[1];
	char str[1000];

	SharedMemory SM(name, n, 1, 2);

	while (true)
	{
		SM.read_data(A, str);

		cout << A[0] << " Seconds";
		Sleep(1000);
	}


	cout << "\ndone.\n";
	system("pause");

	return 0;
}

