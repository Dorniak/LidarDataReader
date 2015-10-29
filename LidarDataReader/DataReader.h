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

	cli::array<Double>^ InterpolateAzimuth(cli::array<Byte>^& ReceiveBytes);

	cli::array<Double>^ ExtractDistances(cli::array<Byte>^& ReceiveBytes);

	cli::array<Double>^ ExtractIntensities(cli::array<Byte>^& ReceiveBytes);

	double getAngle(int channel);

private:

	/*Client for the UDP connection*/
	UdpClient^ ClientLIDAR;

	/*Client configuration object.Also here is save the info about client(LIDAR)*/
	IPEndPoint^ LaserIpEndPoint;

};

