class SharedMemory
{

public:

	//Vars
	char* Name;//Name of Shared Memory
	int Size;//Size of Shared Memory
	char* P;//Pointer to Shared Memory
	int q;//Unique Identifier
	int N;

	//Funcs
	SharedMemory(char* Name, int Size, int q, int N);

	int write_data(double* A, char* str);

	int read_data(double* A, char* str);



};