#include "DataReader.h"
#include <time.h>
#define MIN_DISTANCE 1
#define UDP_FLAG 2
DataReader::DataReader(IPEndPoint^ LIp)//SerialPort^ p
{
	/*Constructor of the class, here we can configure the port where our client is going to listen
	We listen for any IP on port 2368*/
	LaserIpEndPoint = LIp;
	ClientLIDAR = gcnew UdpClient(LaserIpEndPoint);
}
DataReader::~DataReader()
{
	delete ClientLIDAR;
	delete LaserIpEndPoint;
}

void DataReader::ReadData()
{
	/*Function that read data from the LIDAR by the network
	We receive a UDP package where we only need the first 1200 bytes as a points information less
	the 42 header bytes. Then we convert and save the info according to the points and finally we save the point
	in the cloud vector.*/

	float azimuth = 0, distance = 0, intensity = 0, first_azimuth = -1;
	cli::array<Byte>^ ReceiveBytes;
	int bytesread = 0, channel = 0;
	clock_t start, finish;
	cliext::vector<Punto3D ^>^ pointCloud = gcnew cliext::vector<Punto3D ^>;

	while (1) {
	
		ReceiveBytes = ClientLIDAR->Receive(LaserIpEndPoint);
		start = clock();

		for (int block = 1; block <= 12; block++) {
			bytesread += UDP_FLAG;

			if (first_azimuth == -1) {
				first_azimuth = (ReceiveBytes[bytesread] + (ReceiveBytes[bytesread + 1] << 8));
				first_azimuth /= 100;
			}
			else {
				azimuth = (ReceiveBytes[bytesread] + (ReceiveBytes[bytesread + 1] << 8));
				azimuth /= 100;
			}
			bytesread += 2;

			if (azimuth == first_azimuth) {
				//enviar vector
				pointCloud->clear();
				first_azimuth = azimuth;
			}

			for (int i = 0; i < 32;i++) {
				distance = (ReceiveBytes[bytesread] + (ReceiveBytes[bytesread + 1] << 8));
				bytesread += 2;
				intensity = ReceiveBytes[bytesread];
				bytesread++;
				channel++;

				if (distance >= MIN_DISTANCE)
				{
					pointCloud->push_back(gcnew Punto3D(distance, intensity, azimuth));
				}

				if (channel > 15)
				{
					channel = 0;
				}
			}
		}
		bytesread = 0;
		channel = 0;
		finish = clock();
		double duration = (double)(finish - start) / CLOCKS_PER_SEC;
		Console::WriteLine("Duracion: {0}, paquetes por S: {1} puntos {2}", duration, 60 / duration, pointCloud->size());
	}//While
}