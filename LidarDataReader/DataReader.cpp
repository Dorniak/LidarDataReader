#include "DataReader.h"
#include <time.h>
#define MIN_DISTANCE 1
#define UDP_FLAG 2
#define NUMBER_OF_BLOCKS 12
#define NUMBER_OF_CHANNELS 32
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
	clock_t start_procces, finish_process,start_loop,finish_loop;
	cliext::vector<Punto3D ^>^ pointCloud = gcnew cliext::vector<Punto3D ^>;
	
	while (true) {
		start_loop = clock();

		ReceiveBytes = ClientLIDAR->Receive(LaserIpEndPoint);

		start_procces = clock();
		for (int block = 1; block <= NUMBER_OF_BLOCKS; block++) {
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

			for (int i = 0; i < NUMBER_OF_CHANNELS;i++) {
				distance = (ReceiveBytes[bytesread] + (ReceiveBytes[bytesread + 1] << 8));
				bytesread += 2;
				intensity = ReceiveBytes[bytesread];
				bytesread++;
				channel++;
				pointCloud->push_back(gcnew Punto3D((distance*2)/1000, intensity, azimuth));
				if (channel > 15)
				{
					channel = 0;
				}
			}
		}//for blocks
		bytesread = 0;
		channel = 0;
		finish_process = clock();
		finish_loop = clock();
		double duration = (double)(finish_process - start_procces) / CLOCKS_PER_SEC;
		double duration2 = (double)(finish_loop - start_loop) / CLOCKS_PER_SEC;
		Console::Write("\r|\t{0}\t|\t{1}\t|\t{2}\t|\t{3}\t|", duration, 60 / duration, pointCloud->size(), duration2);
		
	}//While
}