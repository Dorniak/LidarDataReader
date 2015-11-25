#pragma once
#pragma warning( disable : 4244 ) //Life is not perfect
#include "Punto3D.h"
#include <cliext/vector>


using namespace std;
using namespace System;
using namespace System::Net;
using namespace System::Text;
using namespace System::Net::Sockets;
using namespace System::IO;
using namespace System::Text;
using namespace System::Threading;

using namespace System::Collections::Generic;
ref class DataReader
{

public:
	DataReader(IPEndPoint^ LaserIpEndPoint);
	~DataReader();

	Object^ data1;
	void ReadData(Object^ data);
	void StopReadData();
	void ReadDataThread();
	double getProcessTime();
	double getPackageTime();
	double getAngle(int channel);

	void saveProcessTime(double time);
	void savePackageTime(double time);
	cli::array<Double>^ InterpolateAzimuth(cli::array<Byte>^& ReceiveBytes);
	cli::array<Double>^ ExtractDistances(cli::array<Byte>^& ReceiveBytes);
	cli::array<Double>^ ExtractIntensities(cli::array<Byte>^& ReceiveBytes);



private:

	/*Client for the UDP connection*/
	UdpClient^ ClientLIDAR;

	/*Client configuration object.Also here is save the info about client(LIDAR)*/
	IPEndPoint^ LaserIpEndPoint;
	double process_Time;
	double package_Time;
	Thread^ thread_reader;
};

