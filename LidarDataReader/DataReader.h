#pragma once
#pragma warning( disable : 4244 ) //Life is not perfect
#include "Punto3D.h"
#include <cliext/vector>


using namespace std;
using namespace System;
using namespace System::Net;
using namespace System::Text;
using namespace System::Net::Sockets;
using namespace cliext;
using namespace System::Collections::Generic;
ref class DataReader
{

public:
	DataReader(IPEndPoint^ LaserIpEndPoint);
	~DataReader();
	void ReadData();

private:

	/*Client for the UDP connection*/
	UdpClient^ ClientLIDAR;

	/*Client configuration object.Also here is save the info about client(LIDAR)*/
	IPEndPoint^ LaserIpEndPoint;

};

