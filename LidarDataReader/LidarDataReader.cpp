// LidarDataReader.cpp: archivo de proyecto principal.

#include "Punto3d.h"
#include "DataReader.h"
using namespace System;
using namespace System::Net;
int main(cli::array<System::String ^> ^args)
{

    Console::WriteLine(L"Lidar Ready!\n");
	
	IPEndPoint^ LaserIpEndPoint = gcnew IPEndPoint(IPAddress::Any, 2368);
	DataReader^ reader = gcnew DataReader(LaserIpEndPoint);
	
	Console::WriteLine("| Process  Time |\tPackages/s\t|\tPoints\t|\tLoop  Time|");
		reader->ReadData();
	
	
}
