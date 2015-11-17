// LidarDataReader.cpp: archivo de proyecto principal.

#include "Punto3d.h"
#include "DataReader.h"
using namespace System;
using namespace System::Net;
int main(cli::array<System::String ^> ^args)
{
	
	
	IPEndPoint^ LaserIpEndPoint = gcnew IPEndPoint(IPAddress::Any, 2368);
	DataReader^ reader = gcnew DataReader(LaserIpEndPoint);
	Console::SetWindowSize(80, 50);
	Console::Beep(415,200);
	Console::WriteLine("| Process  Time |\tPackages/s\t|\tPoints\t|\tLoop  Time|\n");
	double frecuency, packages, ptime;
	bool treal;
		reader->ReadData(frecuency,packages,ptime,treal);
	
	
}
