// LidarDataReader.cpp: archivo de proyecto principal.

#include "Punto3d.h"
#include "DataReader.h"
using namespace System;
using namespace System::Net;
int main(cli::array<System::String ^> ^args)
{

    Console::WriteLine(L"Hola a todos");

	IPEndPoint^ LaserIpEndPoint = gcnew IPEndPoint(IPAddress::Any, 2368);
	DataReader^ reader = gcnew DataReader(LaserIpEndPoint);
	
		
		reader->ReadData();
	
	
}
